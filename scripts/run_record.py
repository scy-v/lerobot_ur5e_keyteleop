import yaml
from typing import Dict, Any
from lerobot_robot_ur5e import UR5eConfig, UR5e
from lerobot_teleoperator_ur5e import UR5eTeleopConfig, UR5eTeleop
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense.camera_realsense import RealSenseCameraConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.utils import log_say
from lerobot.processor import make_default_processors
from lerobot.utils.visualization_utils import init_rerun
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
import logging
logging.basicConfig(level=logging.WARNING)

class RecordConfig:
    def __init__(self, cfg: Dict[str, Any]):
        storage = cfg["storage"]
        task = cfg["task"]
        time = cfg["time"]
        cam = cfg["cameras"]
        robot = cfg["robot"]

        # global config
        self.repo_id: str = cfg["repo_id"]
        self.fps: str = cfg.get("fps", 15)

        # robot config
        self.robot_ip: str = robot["ip"]
        self.gripper_port: str = robot["gripper_port"]

        # task config
        self.num_episodes: int = task.get("num_episodes", 1)
        self.display: bool = task.get("display", True)
        self.task_description: str = task.get("task_description", "default task")

        # time config
        self.episode_time_sec: int = time.get("episode_time_sec", 60)
        self.reset_time_sec: int = time.get("reset_time_sec", 10)
        self.save_mera_period: int = time.get("save_mera_period", 1)

        # cameras config
        self.wrist_cam_serial: str = cam["wrist_cam_serial"]
        self.exterior_cam_serial: str = cam["exterior_cam_serial"]

        # storage config
        self.push_to_hub: bool = storage.get("push_to_hub", False)


def main(record_cfg: RecordConfig):
    # Create RealSenseCamera configurations
    wrist_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.wrist_cam_serial,
                                    fps=record_cfg.fps,
                                    width=640,
                                    height=480,
                                    color_mode=ColorMode.RGB,
                                    use_depth=False,
                                    rotation=Cv2Rotation.NO_ROTATION)

    exterior_image_cfg = RealSenseCameraConfig(serial_number_or_name=record_cfg.exterior_cam_serial,
                                    fps=record_cfg.fps,
                                    width=640,
                                    height=480,
                                    color_mode=ColorMode.RGB,
                                    use_depth=False,
                                    rotation=Cv2Rotation.NO_ROTATION)

    # Create the robot and teleoperator configurations
    camera_config = {"wrist_image": wrist_image_cfg, "exterior_image": exterior_image_cfg}
    teleop_config = UR5eTeleopConfig(use_gripper=True)
    robot_config = UR5eConfig(
        robot_ip=record_cfg.robot_ip,
        gripper_port=record_cfg.gripper_port,
        cameras = camera_config
    )
    # Initialize the robot and teleoperator
    robot = UR5e(robot_config)
    teleop = UR5eTeleop(teleop_config)

    # Configure the dataset features
    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation", use_video=True)
    dataset_features = {**action_features, **obs_features}

    # # Create the dataset
    dataset = LeRobotDataset.create(
        repo_id=record_cfg.repo_id,
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
    teleop.connect()

    episode_idx = 0
    while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
        log_say(f"Recording episode {episode_idx + 1} of {record_cfg.num_episodes}", play_sounds=False)
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

        # Reset the environment if not stopping or re-recording
        if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
            log_say("Reset the environment", play_sounds=False)
            record_loop(
                robot=robot,
                events=events,
                fps=record_cfg.fps,
                teleop=teleop,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                control_time_s=record_cfg.reset_time_sec,
                single_task=record_cfg.task_description,
                display_data=record_cfg.display,
            )

        if events["rerecord_episode"]:
            log_say("Re-recording episode", play_sounds=False)
            events["rerecord_episode"] = False
            events["exit_early"] = False
            dataset.clear_episode_buffer()
            continue

        dataset.save_episode()
        episode_idx += 1

    # Clean up
    log_say("Stop recording", play_sounds=False)
    robot.disconnect()
    teleop.disconnect()
    dataset.finalize()
    if record_cfg.push_to_hub:
        dataset.push_to_hub()

if __name__ == "__main__":
    with open("./config/cfg.yaml", 'r') as f:
        cfg = yaml.safe_load(f)

    record_cfg = RecordConfig(cfg["record"])
    main(record_cfg)