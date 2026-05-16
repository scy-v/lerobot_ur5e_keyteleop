import yaml
from pathlib import Path
from typing import Dict, Any
from scripts.utils.dataset_utils import generate_dataset_name, update_dataset_info
from lerobot_robot_ur5e import UR5eConfig, UR5e
from lerobot_teleoperator_ur5e import UR5eTeleopConfig, UR5eTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
import shutil
import termios
import sys
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.utils.control_utils import sanity_check_dataset_robot_compatibility
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")

class RecordConfig:
    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]
        teleop = cfg.get("teleop", {})
        force_cfg = robot.get("force_mode", {})
        position_cfg = robot.get("position_mode", {})
        payload_cfg = robot.get("payload", {})

        # global config
        self.repo_id: str = cfg["repo_id"]
        self.fps: str = cfg.get("fps", 15)
        self.dataset_path: str = HF_LEROBOT_HOME / self.repo_id
        self.user_info: str = cfg.get("user_notes", None)

        # robot config
        self.robot_ip: str = robot["ip"]
        self.gripper_port: str = robot["gripper_port"]
        self.use_gripper: bool = robot.get("use_gripper", True)
        self.close_threshold: float = robot.get("close_threshold", 0.7)
        self.gripper_reverse: bool = robot.get("gripper_reverse", False)
        self.gripper_bin_threshold: float = robot.get("gripper_bin_threshold", 0.98)
        self.gripper_force: int = robot.get("gripper_force", 70)
        self.gripper_speed: int = robot.get("gripper_speed", 60)
        self.control_space: str = robot.get("control_space", "position")
        self.reference_frame: str = robot.get("reference_frame", "base")
        self.debug: bool = robot.get("debug", False)
        self.kp: int = force_cfg.get("kp", 2000)
        self.kd: int = force_cfg.get("kd", 200)
        self.kp_rot: int = force_cfg.get("kp_rot", 4000)
        self.kd_rot: int = force_cfg.get("kd_rot", 800)
        self.rtde_freq: int = force_cfg.get("rtde_freq", 125)
        self.select_vector: list = force_cfg.get("select_vector", [1, 1, 1, 1, 1, 1])
        self.force_limit: list = force_cfg.get("force_limit", [2, 2, 2, 2, 2, 2])
        self.pos_delta: float = force_cfg.get("pos_delta", 0.2)
        self.vel_delta: float = force_cfg.get("vel_delta", 0.2)
        self.gain_scale: float = force_cfg.get("gain_scale", 1.5)
        self.speed: float = position_cfg.get("speed", 0.5)
        self.acceleration: float = position_cfg.get("acceleration", 0.5)
        self.servo_time: float = position_cfg.get("servo_time", 0.1)
        self.lookahead_time: float = position_cfg.get("lookahead_time", 0.1)
        self.gain: int = position_cfg.get("gain", 300)
        self.payload_mass: float = payload_cfg.get("mass", 1.601)
        self.payload_cog: list = payload_cfg.get("cog", [0.011, -0.002, 0.052])
        self.init_pose: list[float] = robot["init_pose"]
        self.init_pos_range: list[float] = robot.get("init_pos_range", [0.0, 0.0, 0.0])

        # teleop config
        self.teleop_position_step_size: float = teleop.get("position_step_size", teleop.get("step_size", 0.01))
        self.teleop_force_step_size: float = teleop.get("force_step_size", teleop.get("step_size", 0.048))
        self.teleop_position_rot_step_size: float = teleop.get("position_rot_step_size", teleop.get("rot_step_size", 0.02))
        self.teleop_force_rot_step_size: float = teleop.get("force_rot_step_size", teleop.get("rot_step_size", 0.05))
        if self.control_space == "position":
            self.teleop_step_size: float = self.teleop_position_step_size
            self.teleop_rot_step_size: float = self.teleop_position_rot_step_size
        elif self.control_space == "force":
            self.teleop_step_size: float = self.teleop_force_step_size
            self.teleop_rot_step_size: float = self.teleop_force_rot_step_size
        else:
            raise ValueError(f"Unsupported control_space: {self.control_space}")

        # task config
        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("description", task.get("task_description", "default task"))
        self.resume: bool = task.get("resume", False)
        self.resume_dataset: str = task.get("resume_dataset", self.repo_id)
        
        # time config
        self.episode_time_sec: int = time.get("episode_time_sec", 60)
        self.reset_time_sec: int = time.get("reset_time_sec", 10)
        self.save_mera_period: int = time.get("save_meta_period", time.get("save_mera_period", 1))

        # cameras config
        self.wrist_cam_serial: str = cam["wrist_cam_serial"]
        self.exterior_cam_serial: str = cam["exterior_cam_serial"]
        self.width: int = cam.get("width", 640)
        self.height: int = cam.get("height", 480)

        # storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)


def handle_incomplete_dataset(dataset_path):
    if dataset_path.exists():
        print(f"====== [WARNING] Detected an incomplete dataset folder: {dataset_path} ======")
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        ans = input("Do you want to delete it? (y/n): ").strip().lower()
        if ans == "y":
            print(f"====== [DELETE] Removing folder: {dataset_path} ======")
            shutil.rmtree(dataset_path, ignore_errors=True)
            print("====== [DONE] Incomplete dataset folder deleted successfully. ======")
        else:
            print("====== [KEEP] Incomplete dataset folder retained, please check manually. ======")


def wait_for_enter(prompt: str) -> None:
    while True:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        user_input = input(prompt)
        if user_input == "":
            return
        logging.info("====== [WARNING] Please press only Enter to continue ======")


def run_record(record_cfg: RecordConfig):
    robot = None
    teleop = None
    dataset = None
    dataset_name = None

    try:
        dataset_name, data_version = generate_dataset_name(record_cfg)

        # Create RealSenseCamera configurations
        wrist_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.wrist_cam_serial,
                                        fps=record_cfg.fps,
                                        width=record_cfg.width,
                                        height=record_cfg.height,
                                        color_mode=ColorMode.RGB,
                                        use_depth=False,
                                        rotation=Cv2Rotation.NO_ROTATION)

        exterior_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.exterior_cam_serial,
                                        fps=record_cfg.fps,
                                        width=record_cfg.width,
                                        height=record_cfg.height,
                                        color_mode=ColorMode.RGB,
                                        use_depth=False,
                                        rotation=Cv2Rotation.NO_ROTATION)

        # Create the robot and teleoperator configurations
        camera_config = {"wrist_image": wrist_image_cfg, "exterior_image": exterior_image_cfg}
        teleop_config = UR5eTeleopConfig(
            use_gripper=record_cfg.use_gripper,
            step_size=record_cfg.teleop_step_size,
            rot_step_size=record_cfg.teleop_rot_step_size,
            reference_frame=record_cfg.reference_frame,
        )
        robot_config = UR5eConfig(
            robot_ip=record_cfg.robot_ip,
            gripper_port=record_cfg.gripper_port,
            cameras=camera_config,
            use_gripper=record_cfg.use_gripper,
            close_threshold=record_cfg.close_threshold,
            gripper_reverse=record_cfg.gripper_reverse,
            gripper_bin_threshold=record_cfg.gripper_bin_threshold,
            gripper_force=record_cfg.gripper_force,
            gripper_speed=record_cfg.gripper_speed,
            control_space=record_cfg.control_space,
            debug=record_cfg.debug,
            kp=record_cfg.kp,
            kd=record_cfg.kd,
            kp_rot=record_cfg.kp_rot,
            kd_rot=record_cfg.kd_rot,
            rtde_freq=record_cfg.rtde_freq,
            select_vector=record_cfg.select_vector,
            force_limit=record_cfg.force_limit,
            pos_delta=record_cfg.pos_delta,
            vel_delta=record_cfg.vel_delta,
            gain_scale=record_cfg.gain_scale,
            speed=record_cfg.speed,
            acceleration=record_cfg.acceleration,
            servo_time=record_cfg.servo_time,
            lookahead_time=record_cfg.lookahead_time,
            gain=record_cfg.gain,
            payload_mass=record_cfg.payload_mass,
            payload_cog=record_cfg.payload_cog,
            init_pose=record_cfg.init_pose,
            init_pos_range=record_cfg.init_pos_range,
            reference_frame=record_cfg.reference_frame,
        )
        # Initialize the robot and teleoperator
        robot = UR5e(robot_config)
        teleop = UR5eTeleop(teleop_config)
        teleop.set_robot(robot)

        # Configure the dataset features
        action_features = hw_to_dataset_features(robot.action_features, "action")
        obs_features = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
        dataset_features = {**action_features, **obs_features}

        if record_cfg.resume:
            dataset = LeRobotDataset(
                dataset_name,
            )

            if hasattr(robot, "cameras") and len(robot.cameras) > 0:
                dataset.start_image_writer()
            sanity_check_dataset_robot_compatibility(dataset, robot, record_cfg.fps, dataset_features)
        else:
            dataset = LeRobotDataset.create(
                repo_id=dataset_name,
                fps=record_cfg.fps,
                features=dataset_features,
                robot_type=robot.name,
                use_videos=True,
                image_writer_threads=4,
            )

        # Set the episode metadata buffer size to 1, so that each episode is saved immediately
        dataset.meta.metadata_buffer_size = record_cfg.save_mera_period

        # Initialize the keyboard listener and rerun visualization
        _, events = init_keyboard_listener()
        init_rerun(session_name="recording")

        # Create processor
        teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

        robot.connect()
        robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pos_range)
        teleop.connect()

        episode_idx = 0
        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            robot.set_episode_reference_pose()
            logging.info(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")
            record_loop(
                robot=robot,
                events=events,
                fps=record_cfg.fps,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                control_time_s=record_cfg.episode_time_sec,
                single_task=record_cfg.task_description,
                display_data=record_cfg.display,
            )

            if events["rerecord_episode"]:
                logging.info("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pos_range)
                continue
            robot.stop_force()
            dataset.save_episode()

            # Reset the environment if not stopping or re-recording
            if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
                wait_for_enter("====== [WAIT] Press Enter to reset the robot ======")
                logging.info("====== [RESET] Resetting the environment ======")
                robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pos_range)
                wait_for_enter("====== [WAIT] Press Enter to start the next episode ======")

            episode_idx += 1

        # Clean up
        logging.info("Stop recording")
        robot.disconnect()
        teleop.disconnect()
        dataset.finalize()

        update_dataset_info(record_cfg, dataset_name, data_version)
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except (Exception, KeyboardInterrupt) as e:
        logging.info(f"====== [ERROR] {e} ======" if isinstance(e, Exception) else "\n====== [INFO] Ctrl+C detected ======")
        if robot is not None:
            robot.disconnect()
        if teleop is not None:
            teleop.disconnect()
        if dataset_name is not None:
            dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
            handle_incomplete_dataset(dataset_path)

def main():
    with open(Path(__file__).parents[1] / "config" / "cfg.yaml", 'r') as f:
        cfg = yaml.safe_load(f)

    record_cfg = RecordConfig(cfg["record"])
    run_record(record_cfg)


if __name__ == "__main__":
    main()
