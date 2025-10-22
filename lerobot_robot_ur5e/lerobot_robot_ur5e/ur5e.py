import logging
import time
from typing import Any
import threading
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

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
        self.gripper_control = 1

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self.name} is already connected.")

        # Connect to robot
        self._arm['rtde_r'], self._arm['rtde_c'] = self._check_ur5e_connection(self.config.robot_ip)

        # Initialize gripper
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
        gripper = PGE(port)
        gripper.init_feedback()
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
            if self.gripper_control != 1:
                print(int(1000 * self.gripper_control // 2))
                self._gripper.set_pos(val=int(1000 * self.gripper_control // 2), blocking=False)
            gripper_pos = self._gripper.read_pos()
            self._gripper.pos = gripper_pos

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
            "tcp_pose.x": float,
            "tcp_pose.y": float,
            "tcp_pose.z": float,
            "tcp_pose.r": float,
            "tcp_pose.p": float,
            "tcp_pose.y": float,
            "gripper_position": float,
        }

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "delta_x": float,
            "delta_y": float,
            "delta_z": float,
            "gripper_position": float,
        }

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if "joint_position" in action:
            joint_position = action["joint_position"]
            if len(joint_position) != 6:
                raise ValueError(f"joint_position must have 6 values, got {len(joint_position)}")
            self._arm["rtde_c"].servoJ(joint_position)

        if "delta_x" in action and "delta_y" in action and "delta_z" in action and "joint_position" not in action:
            tcp_pose = self._arm["rtde_r"].getActualTCPPose()
            speed = 0.5           # m/s
            acceleration = 0.5     # m/s^2
            time = 0.1            # 通常0即可
            lookahead_time = 0.1   # 轨迹预见时间
            gain = 300             # 控制增益
            tcp_pose = [
                tcp_pose[0] + action["delta_x"],
                tcp_pose[1] + action["delta_y"],
                tcp_pose[2] + action["delta_z"],
                tcp_pose[3],  # 保持当前姿态
                tcp_pose[4],
                tcp_pose[5],
            ]

            t_start = self._arm["rtde_c"].initPeriod()

            self._arm["rtde_c"].servoL(tcp_pose, speed, acceleration, time, lookahead_time, gain)

        if "gripper_position" in action:
            self.gripper_control = action["gripper_position"]

        self._arm["rtde_c"].waitPeriod(t_start)

        return action

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read joint positions
        joint_position = self._arm["rtde_r"].getActualQ()

        # Read tcp pose
        tcp_pose = self._arm["rtde_r"].getActualTCPPose()
        
        obs_dict = {}

        for i in range(len(joint_position)):
            obs_dict[f"joint_{i+1}.pos"] = joint_position[i]

        for i, axis in enumerate(["x", "y", "z","r","p","y"]):
            obs_dict[f"tcp_pose.{axis}"] = tcp_pose[i]

        obs_dict["gripper_position"] = self._gripper.pos / 1000.0

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        self._prev_observation = obs_dict

        return obs_dict

    def disconnect(self) -> None:
        if not self.is_connected:
            return

        if self._arm is not None:
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
