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
from send2trash import send2trash
import termios
import sys
import time as time_module
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
        self.init_gripper: bool = robot.get("init_gripper", True)
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


def handle_incomplete_dataset(dataset_path) -> bool:
    if dataset_path.exists():
        print(f"====== [WARNING] Detected an incomplete dataset folder: {dataset_path} ======")
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        ans = input("Do you want to delete it? (y/n): ").strip().lower()
        if ans == "y":
            print(f"====== [TRASH] Moving folder to trash: {dataset_path} ======")
            send2trash(str(dataset_path))
            print("====== [DONE] Incomplete dataset folder moved to trash successfully. ======")
            return False
        else:
            print("====== [KEEP] Incomplete dataset folder retained, please check manually. ======")
            return True
    return False


def format_duration(seconds: float) -> str:
    total_seconds = max(0, int(round(seconds)))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours:
        return f"{hours}h {minutes}m {seconds}s"
    if minutes:
        return f"{minutes}m {seconds}s"
    return f"{seconds}s"


def append_record_times(
    record_cfg: RecordConfig,
    record_time: str,
    reset_time: str,
    total_time: str,
    avg_record_time: str,
    avg_reset_time: str,
) -> None:
    duration_info = (
        f"record_time={record_time}, reset_time={reset_time}, total_time={total_time}, "
        f"avg_record_time={avg_record_time}, avg_reset_time={avg_reset_time}"
    )
    if record_cfg.user_info:
        if duration_info not in str(record_cfg.user_info):
            record_cfg.user_info = f"{record_cfg.user_info}; {duration_info}"
    else:
        record_cfg.user_info = duration_info


def get_episode_buffer_size(dataset: LeRobotDataset | None) -> int:
    if dataset is None:
        return 0
    episode_buffer = getattr(dataset, "episode_buffer", None)
    if not episode_buffer:
        return 0
    return int(episode_buffer.get("size", 0))


def discard_unsaved_episode(dataset: LeRobotDataset | None) -> None:
    if dataset is None:
        return

    if get_episode_buffer_size(dataset) <= 0:
        return

    try:
        dataset.clear_episode_buffer(delete_images=len(dataset.meta.image_keys) > 0)
        logging.info("====== [INFO] Discarded unsaved episode buffer. ======")
    except Exception as cleanup_error:
        logging.info(f"====== [WARNING] Failed to discard unsaved episode buffer: {cleanup_error} ======")


def finalize_dataset_safely(dataset: LeRobotDataset | None) -> None:
    if dataset is None:
        return

    try:
        dataset.finalize()
    except Exception as finalize_error:
        logging.info(f"====== [WARNING] Failed to finalize dataset cleanly: {finalize_error} ======")


def wait_for_enter(prompt: str) -> None:
    while True:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        user_input = input(prompt)
        if user_input == "":
            return
        logging.info("====== [WARNING] Please press only Enter to continue ======")


def reset_to_init_pose(
    record_cfg: RecordConfig,
    robot: UR5e,
    teleop: UR5eTeleop,
    events: dict,
    teleop_action_processor,
    robot_action_processor,
    robot_observation_processor,
) -> None:
    logging.info("====== [RESET] Use keyboard teleoperation to move the robot to a safe pre-reset pose ======")
    logging.info("\033[32m====== [RESET] Press the right arrow key to finish manual reset control ======\033[0m")
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

    events["exit_early"] = False
    events["rerecord_episode"] = False
    if events["stop_recording"]:
        return

    logging.info("====== [RESET] Automatically resetting to configured initial TCP pose ======")
    robot.reset_to_init_pose(record_cfg.init_pose, record_cfg.init_pos_range)


def run_record(record_cfg: RecordConfig):
    robot = None
    teleop = None
    dataset = None
    dataset_name = None
    data_version = None
    record_start_time = None
    record_loop_time_s = 0.0
    record_loop_count = 0
    dataset_info_updated = False

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
            init_gripper=record_cfg.init_gripper,
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
        record_start_time = time_module.perf_counter()
        while episode_idx < record_cfg.num_episodes and not events["stop_recording"]:
            events["exit_early"] = False
            events["rerecord_episode"] = False
            robot.set_episode_reference_pose()
            logging.info(f"====== [RECORD] Recording episode {episode_idx + 1} of {record_cfg.num_episodes} ======")
            episode_record_start = time_module.perf_counter()
            try:
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
            finally:
                record_loop_time_s += time_module.perf_counter() - episode_record_start
                record_loop_count += 1

            if events["rerecord_episode"]:
                logging.info("Re-recording episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                reset_to_init_pose(
                    record_cfg,
                    robot,
                    teleop,
                    events,
                    teleop_action_processor,
                    robot_action_processor,
                    robot_observation_processor,
                )
                continue
            robot.stop_force()

            if get_episode_buffer_size(dataset) <= 0:
                logging.info("====== [WARNING] No frames were recorded for this episode; skipping save. ======")
                events["exit_early"] = False
                events["rerecord_episode"] = False
                if events["stop_recording"]:
                    break
                continue

            dataset.save_episode()

            # Reset the environment if not stopping or re-recording
            if not events["stop_recording"] and (episode_idx < record_cfg.num_episodes - 1 or events["rerecord_episode"]):
                wait_for_enter("====== [WAIT] Press Enter to reset the robot ======")
                reset_to_init_pose(
                    record_cfg,
                    robot,
                    teleop,
                    events,
                    teleop_action_processor,
                    robot_action_processor,
                    robot_observation_processor,
                )
                if not events["stop_recording"]:
                    wait_for_enter("====== [WAIT] Press Enter to start the next episode ======")

            episode_idx += 1

        # Clean up
        logging.info("Stop recording")
        robot.disconnect()
        teleop.disconnect()
        dataset.finalize()

        total_time_s = time_module.perf_counter() - record_start_time
        reset_time_s = max(0.0, total_time_s - record_loop_time_s)
        record_duration = format_duration(record_loop_time_s)
        reset_duration = format_duration(reset_time_s)
        total_duration = format_duration(total_time_s)
        avg_denominator = max(record_loop_count, 1)
        avg_record_duration = format_duration(record_loop_time_s / avg_denominator)
        avg_reset_duration = format_duration(reset_time_s / avg_denominator)
        logging.info(f"====== [INFO] Record loop time: {record_duration} ======")
        logging.info(f"====== [INFO] Reset/non-record time: {reset_duration} ======")
        logging.info(f"====== [INFO] Total recording time: {total_duration} ======")
        logging.info(f"====== [INFO] Average record loop time: {avg_record_duration} ======")
        logging.info(f"====== [INFO] Average reset/non-record time: {avg_reset_duration} ======")
        append_record_times(
            record_cfg,
            record_duration,
            reset_duration,
            total_duration,
            avg_record_duration,
            avg_reset_duration,
        )
        update_dataset_info(record_cfg, dataset_name, data_version)
        dataset_info_updated = True
        if record_cfg.push_to_hub:
            dataset.push_to_hub()

    except (Exception, KeyboardInterrupt) as e:
        logging.info(f"====== [ERROR] {e} ======" if isinstance(e, Exception) else "\n====== [INFO] Ctrl+C detected ======")
        if record_start_time is not None:
            total_time_s = time_module.perf_counter() - record_start_time
            reset_time_s = max(0.0, total_time_s - record_loop_time_s)
            record_duration = format_duration(record_loop_time_s)
            reset_duration = format_duration(reset_time_s)
            total_duration = format_duration(total_time_s)
            avg_denominator = max(record_loop_count, 1)
            avg_record_duration = format_duration(record_loop_time_s / avg_denominator)
            avg_reset_duration = format_duration(reset_time_s / avg_denominator)
            logging.info(f"====== [INFO] Record loop time: {record_duration} ======")
            logging.info(f"====== [INFO] Reset/non-record time: {reset_duration} ======")
            logging.info(f"====== [INFO] Total recording time: {total_duration} ======")
            logging.info(f"====== [INFO] Average record loop time: {avg_record_duration} ======")
            logging.info(f"====== [INFO] Average reset/non-record time: {avg_reset_duration} ======")
            append_record_times(
                record_cfg,
                record_duration,
                reset_duration,
                total_duration,
                avg_record_duration,
                avg_reset_duration,
            )
        if robot is not None:
            robot.disconnect()
        if teleop is not None:
            teleop.disconnect()
        discard_unsaved_episode(dataset)
        finalize_dataset_safely(dataset)
        if dataset_name is not None:
            dataset_path = Path(HF_LEROBOT_HOME) / dataset_name
            keep_dataset = handle_incomplete_dataset(dataset_path)
            if keep_dataset and data_version is not None and not dataset_info_updated:
                update_dataset_info(record_cfg, dataset_name, data_version)

def main():
    with open(Path(__file__).parents[1] / "config" / "cfg.yaml", 'r') as f:
        cfg = yaml.safe_load(f)

    record_cfg = RecordConfig(cfg["record"])
    run_record(record_cfg)


if __name__ == "__main__":
    main()
