import logging
import time
from typing import Any
import threading
import serial
import crcmod
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

import numpy as np
from scipy.spatial.transform import Rotation as R

from lerobot.cameras import make_cameras_from_configs
from lerobot.utils.errors import DeviceNotConnectedError, DeviceAlreadyConnectedError
from lerobot.robots.robot import Robot
from pyDHgripper import PGE
from .config_ur5e import UR5eConfig

logger = logging.getLogger(__name__)

class UR5e(Robot):
    config_class = UR5eConfig
    name = "ur5e"

    def __init__(self, config: UR5eConfig):
        super().__init__(config)
        self.cameras = make_cameras_from_configs(config.cameras)

        self.config = config
        self._is_connected = False
        self._arm = {}
        self._gripper = None
        self._initial_pose = None
        self._prev_observation = None
        self._gripper_force = config.gripper_force
        self._gripper_speed = config.gripper_speed
        self._gripper_position = 1.0
        self._last_gripper_position = 1.0
        self._episode_reference_ee_pose = None
        self.task_frame = [0, 0, 0, 0, 0, 0]
        self.type = 2
        self._send_action_freq_t = time.perf_counter()
        self._send_action_freq_count = 0

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        if self.config.control_space not in ("position", "force"):
            raise ValueError(f"Unsupported control_space: {self.config.control_space}")
        if self.config.reference_frame not in ("base", "tcp"):
            raise ValueError(f"Unsupported reference_frame: {self.config.reference_frame}")

        # Connect to robot
        self._arm['rtde_r'], self._arm['rtde_c'] = self._check_ur5e_connection(self.config.robot_ip)

        if self.config.control_space == "force":
            self._arm["rtde_c"].forceModeSetGainScaling(self.config.gain_scale)

        self._arm["rtde_c"].setPayload(self.config.payload_mass, self.config.payload_cog)

        # Initialize gripper
        if self.config.use_gripper:
            self._gripper = self._check_gripper_connection(self.config.gripper_port)

            # Start gripper state reader
            self._start_gripper_state_reader()

        # Connect cameras
        print("\n===== Initializing Cameras =====")
        for cam_name, cam in self.cameras.items():
            cam.connect()
            print(f"[CAM] {cam_name} connected successfully.")
        print("===== Cameras Initialized Successfully =====\n")

        self.is_connected = True
        print(f"[INFO] {self.name} env initialization completed successfully.\n")


    def _check_gripper_connection(self, port: str):
        print("\n[GRIPPER] Initializing gripper...")
        if self.config.init_gripper:
            gripper = PGE(port)
            gripper.init_feedback()
        else:
            gripper = PGE.__new__(PGE)
            gripper.ser = serial.Serial(port=port, baudrate=115200)
            gripper.crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
            print("[GRIPPER] Skipped init_state and init_feedback by config.")
        gripper.set_force(self._gripper_force)
        gripper.set_vel(self._gripper_speed)
        print(f"[GRIPPER] Force: {self._gripper_force}, speed: {self._gripper_speed}")
        print("[GRIPPER] Gripper initialized successfully.\n")
        return gripper


    def _check_ur5e_connection(self, robot_ip: str):
        try:
            print("\n[ROBOT] Connecting to UR5e robot...")
            rtde_r = RTDEReceiveInterface(robot_ip)
            rtde_c = RTDEControlInterface(robot_ip)

            joint_positions = rtde_r.getActualQ()
            if joint_positions is not None and len(joint_positions) == 6:
                formatted_joints = [round(j, 4) for j in joint_positions]
                print(f"[ROBOT] Current joint positions: {formatted_joints}")
                print("[ROBOT] UR5e connected successfully.\n")
            else:
                print("[ERROR] Failed to read joint positions. Check connection or remote control mode.")

        except Exception as e:
            print("[ERROR] Failed to connect to UR5e robot.")
            print(f"Exception: {e}\n")

        return rtde_r, rtde_c

    def _start_gripper_state_reader(self):
        threading.Thread(target=self._read_gripper_state, daemon=True).start()

    def _read_gripper_state(self):
        self._gripper.pos = None
        while True:
            gripper_position = 0.0 if self._gripper_position < self.config.close_threshold else 1.0
            if self.config.gripper_reverse:
                gripper_position = 1 - gripper_position

            if gripper_position != self._last_gripper_position:
                self._gripper.set_pos(val=int(1000 * gripper_position), blocking=False)
                self._last_gripper_position = gripper_position

            gripper_pos = self._gripper.read_pos() / 1000.0
            if self.config.gripper_reverse:
                gripper_pos = 1 - gripper_pos
            self._gripper.pos = gripper_pos
            time.sleep(0.01)

    @property
    def _motors_ft(self) -> dict[str, type]:
        joint_pos_features = {f"joint_{i}.pos": float for i in range(1, 7)}
        gripper_features = {
            "gripper_raw_position": float,
            "gripper_raw_bin": float,
            "gripper_action_bin": float,
        }
        tcp_pose_features = {f"tcp_pose.{axis}": float for axis in ["x", "y", "z", "rx", "ry", "rz"]}
        tcp_vel_features = {f"tcp_speed.{axis}": float for axis in ["x", "y", "z", "rx", "ry", "rz"]}
        tcp_force_features = {f"tcp_force.{axis}": float for axis in ["x", "y", "z"]}
        tcp_torque_features = {f"tcp_force.{axis}": float for axis in ["rx", "ry", "rz"]}
        remaining_features = {
            **{f"joint_{i}.vel": float for i in range(1, 7)},
            **{f"joint_{i}.acc": float for i in range(1, 7)},
            **{f"joint_{i}.force": float for i in range(1, 7)},
            **{f"tcp_acc.{axis}": float for axis in ["x", "y", "z"]},
        }
        return {
            **joint_pos_features,
            **gripper_features,
            **tcp_pose_features,
            **tcp_vel_features,
            **tcp_force_features,
            **tcp_torque_features,
            **remaining_features,
        }

    @property
    def action_features(self) -> dict[str, type]:
        features = {
            "delta_x": float,
            "delta_y": float,
            "delta_z": float,
            "delta_rx": float,
            "delta_ry": float,
            "delta_rz": float,
        }
        if self.config.use_gripper:
            features["gripper_position"] = float
        return features

    def _calculate_force(self, target_pos, curr_pos, curr_vel):
        diff_p = np.clip(np.array(target_pos[:3]) - np.array(curr_pos[:3]), -self.config.pos_delta, self.config.pos_delta)
        diff_d = np.clip(-np.array(curr_vel[:3]), -self.config.vel_delta, self.config.vel_delta)
        force_pos = self.config.kp * diff_p + self.config.kd * diff_d

        target_rot = R.from_rotvec(target_pos[3:]).as_matrix()
        curr_rot = R.from_rotvec(curr_pos[3:]).as_matrix()
        rot_err = R.from_matrix(target_rot @ curr_rot.T).as_rotvec()
        torque = (self.config.kp_rot * rot_err - self.config.kd_rot * np.array(curr_vel[3:])) / self.config.rtde_freq

        return np.concatenate((force_pos, torque))

    def _pose_to_transform(self, pose: list[float] | np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
        transform[:3, 3] = pose[:3]
        return transform

    def _transform_to_pose(self, transform: np.ndarray) -> list[float]:
        return [
            *transform[:3, 3].tolist(),
            *R.from_matrix(transform[:3, :3]).as_rotvec().tolist(),
        ]

    def _get_current_tcp_offset(self) -> list[float]:
        return self._arm["rtde_c"].getTCPOffset()

    def get_ee_pose(self) -> list[float]:
        tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        tcp_offset = self._get_current_tcp_offset()
        return self.tcp_to_ee_pose(tcp_pose, tcp_offset).tolist()

    def _ee_to_tcp_pose(self, ee_pose: list[float] | np.ndarray, tcp_offset: list[float] | np.ndarray) -> list[float]:
        ee_transform = self._pose_to_transform(ee_pose)
        offset_transform = self._pose_to_transform(tcp_offset)
        tcp_transform = ee_transform @ offset_transform
        return self._transform_to_pose(tcp_transform)

    def _target_pose_from_action(self, action: dict[str, Any]) -> list[float]:
        current_tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        tcp_offset = self._get_current_tcp_offset()
        current_ee_pose = self.tcp_to_ee_pose(current_tcp_pose, tcp_offset)
        current_position = np.array(current_ee_pose[:3], dtype=float)
        current_rotation = R.from_rotvec(current_ee_pose[3:]).as_matrix()
        delta_position = np.array(
            [float(action["delta_x"]), float(action["delta_y"]), float(action["delta_z"])],
            dtype=float,
        )
        delta_rotation = R.from_euler(
            "xyz",
            [float(action["delta_rx"]), float(action["delta_ry"]), float(action["delta_rz"])],
        ).as_matrix()

        if self.config.reference_frame == "base":
            target_position = current_position + delta_position
            target_rotation = delta_rotation @ current_rotation
        elif self.config.reference_frame == "tcp":
            target_position = current_position + current_rotation @ delta_position
            target_rotation = current_rotation @ delta_rotation
        else:
            raise ValueError(f"Unsupported reference_frame: {self.config.reference_frame}")

        target_transform = np.eye(4)
        target_transform[:3, :3] = target_rotation
        target_transform[:3, 3] = target_position
        target_ee_pose = self._transform_to_pose(target_transform)
        return self._ee_to_tcp_pose(target_ee_pose, tcp_offset)

    def _calculate_ft_target(self, action: dict[str, Any]) -> list[float]:
        curr_pose = self._arm["rtde_r"].getActualTCPPose()
        curr_vel = self._arm["rtde_r"].getActualTCPSpeed()
        target_pose = self._target_pose_from_action(action)

        return self._calculate_force(target_pose, curr_pose, curr_vel)

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # self._print_send_action_frequency()

        t_start = None

        action_keys = ("delta_x", "delta_y", "delta_z", "delta_rx", "delta_ry", "delta_rz")
        if all(key in action for key in action_keys):
            if not self.config.debug:
                if self.config.control_space == "force":
                    ft_target = self._calculate_ft_target(action)
                    t_start = self._arm["rtde_c"].initPeriod()
                    self._arm["rtde_c"].forceMode(
                        self.task_frame,
                        self.config.select_vector,
                        ft_target,
                        self.type,
                        self.config.force_limit,
                    )
                elif self.config.control_space == "position":
                    target_pose = self._target_pose_from_action(action)

                    t_start = self._arm["rtde_c"].initPeriod()
                    self._arm["rtde_c"].servoL(
                        target_pose,
                        self.config.speed,
                        self.config.acceleration,
                        self.config.servo_time,
                        self.config.lookahead_time,
                        self.config.gain,
                    )
        else:
            raise ValueError(f"Keyboard UR5e action must contain {', '.join(action_keys)}.")

        if "gripper_position" in action:
            self._gripper_position = float(action["gripper_position"])

        if t_start is not None:
            self._arm["rtde_c"].waitPeriod(t_start)

        return action

    def _print_send_action_frequency(self):
        self._send_action_freq_count += 1
        now = time.perf_counter()
        dt = now - self._send_action_freq_t
        if dt >= 1.0:
            print(f"[ROBOT] send_action frequency: {self._send_action_freq_count / dt:.2f} Hz")
            self._send_action_freq_t = now
            self._send_action_freq_count = 0

    def _pose_euler(self, pose: list[float] | np.ndarray) -> np.ndarray:
        pose = np.array(pose, dtype=float)
        pose_euler = np.zeros(6, dtype=float)
        pose_euler[:3] = pose[:3]
        pose_euler[3:] = R.from_rotvec(pose[3:]).as_euler("xyz")
        return pose_euler

    def _relative_pose_euler(self, pose: list[float] | np.ndarray) -> np.ndarray:
        if self._episode_reference_ee_pose is None:
            raise RuntimeError("Episode reference EE pose is not set. Call set_episode_reference_pose() first.")

        reference_transform = self._pose_to_transform(self._episode_reference_ee_pose)
        current_transform = self._pose_to_transform(pose)
        relative_transform = np.linalg.inv(reference_transform) @ current_transform

        relative_pose = np.zeros(6, dtype=float)
        relative_pose[:3] = relative_transform[:3, 3]
        relative_pose[3:] = R.from_matrix(relative_transform[:3, :3]).as_euler("xyz")
        return relative_pose

    def set_episode_reference_pose(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._episode_reference_ee_pose = np.array(self.get_ee_pose(), dtype=float)
        logger.info(f"Set episode reference EE pose: {self._episode_reference_ee_pose.tolist()}")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        joint_position = self._arm["rtde_r"].getActualQ()
        joint_velocity = self._arm["rtde_r"].getActualQd()
        joint_acceleration = self._arm["rtde_r"].getTargetQdd()
        joint_force = self._arm["rtde_c"].getJointTorques()
        tcp_pose = self.get_ee_pose()
        if self.config.reference_frame == "base":
            observation_tcp_pose = self._pose_euler(tcp_pose)
        elif self.config.reference_frame == "tcp":
            observation_tcp_pose = self._relative_pose_euler(tcp_pose)
        else:
            raise ValueError(f"Unsupported reference_frame: {self.config.reference_frame}")
        tcp_speed = self._arm["rtde_r"].getActualTCPSpeed()
        tcp_acceleration = self._arm["rtde_r"].getActualToolAccelerometer()
        tcp_force = self._arm["rtde_r"].getActualTCPForce()
        
        obs_dict = {}

        for i in range(len(joint_position)):
            obs_dict[f"joint_{i+1}.pos"] = joint_position[i]

        if self.config.use_gripper:
            gripper_pos = self._gripper.pos if self._gripper.pos is not None else self._last_gripper_position
            obs_dict["gripper_raw_position"] = gripper_pos
            obs_dict["gripper_raw_bin"] = 0 if gripper_pos <= self.config.gripper_bin_threshold else 1
            obs_dict["gripper_action_bin"] = self._last_gripper_position
        else:
            obs_dict["gripper_raw_position"] = 0.0
            obs_dict["gripper_raw_bin"] = 0.0
            obs_dict["gripper_action_bin"] = 0.0

        axes = ["x", "y", "z", "rx", "ry", "rz"]
        for i, axis in enumerate(axes):
            obs_dict[f"tcp_pose.{axis}"] = observation_tcp_pose[i]

        for i, axis in enumerate(axes):
            obs_dict[f"tcp_speed.{axis}"] = tcp_speed[i]

        for i, axis in enumerate(["x", "y", "z"]):
            obs_dict[f"tcp_force.{axis}"] = tcp_force[i]

        for i, axis in enumerate(["rx", "ry", "rz"], start=3):
            obs_dict[f"tcp_force.{axis}"] = tcp_force[i]

        for i in range(len(joint_position)):
            obs_dict[f"joint_{i+1}.vel"] = joint_velocity[i]
            obs_dict[f"joint_{i+1}.acc"] = joint_acceleration[i]
            obs_dict[f"joint_{i+1}.force"] = joint_force[i]

        for i, axis in enumerate(["x", "y", "z"]):
            obs_dict[f"tcp_acc.{axis}"] = tcp_acceleration[i]

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        self._prev_observation = obs_dict

        return obs_dict

    def tcp_to_ee_pose(self, tcp_pose, tcp_offset):
        t_tcp = np.eye(4)
        t_tcp[:3, :3] = R.from_rotvec(tcp_pose[3:]).as_matrix()
        t_tcp[:3, 3] = tcp_pose[:3]

        t_off = np.eye(4)
        t_off[:3, :3] = R.from_rotvec(tcp_offset[3:]).as_matrix()
        t_off[:3, 3] = tcp_offset[:3]

        t_ee = t_tcp @ np.linalg.inv(t_off)
        ee_pos = t_ee[:3, 3]
        ee_rot = R.from_matrix(t_ee[:3, :3]).as_rotvec()
        return np.concatenate([ee_pos, ee_rot])

    def stop_force(self):
        if self.is_connected and self.config.control_space == "force":
            self._arm["rtde_c"].forceMode(
                self.task_frame,
                [0, 0, 0, 0, 0, 0],
                np.array([0, 0, 0, 0, 0, 0]),
                self.type,
                self.config.force_limit,
            )

    def _sample_init_pose(self, init_pose: list[float], init_pos_range: list[float]) -> list[float]:
        if len(init_pose) != 6:
            raise ValueError(f"init_pose must contain 6 values, got {len(init_pose)}.")
        if len(init_pos_range) != 3:
            raise ValueError(f"init_pos_range must contain 3 values, got {len(init_pos_range)}.")

        target_pose = np.array(init_pose, dtype=float)
        random_range = np.abs(np.array(init_pos_range, dtype=float))
        target_pose[:3] += np.random.uniform(-random_range, random_range)
        return target_pose.tolist()

    def reset_to_init_pose(
        self,
        init_pose: list[float] | None = None,
        init_pos_range: list[float] | None = None,
    ) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        target_pose = self._sample_init_pose(
            init_pose if init_pose is not None else self.config.init_pose,
            init_pos_range if init_pos_range is not None else self.config.init_pos_range,
        )

        logger.info("Resetting to initial TCP pose.")
        if self.config.debug:
            logger.info("Debug mode is enabled; skip robot reset motion.")
            return

        if self.config.control_space == "force":
            self.stop_force()
            if hasattr(self._arm["rtde_c"], "forceModeStop"):
                self._arm["rtde_c"].forceModeStop()
        self._arm["rtde_c"].servoStop()
        self._arm["rtde_c"].moveL(target_pose, self.config.speed, self.config.acceleration)
        logger.info("Reached initial position.")

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._arm is not None:
            if self.config.control_space == "force":
                self.stop_force()
            self._arm["rtde_c"].disconnect()
            self._arm["rtde_r"].disconnect()

        for cam in self.cameras.values():
            cam.disconnect()

        self.is_connected = False
        logger.info("[INFO] ===== All connections have been closed =====")

    def calibrate(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return self.is_connected
    
    def configure(self) -> None:
        pass

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @is_connected.setter
    def is_connected(self, value: bool) -> None:
        self._is_connected = value

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
           cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict[str, Any]:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def cameras(self):
        return self._cameras

    @cameras.setter
    def cameras(self, value):
        self._cameras = value

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, value):
        self._config = value
