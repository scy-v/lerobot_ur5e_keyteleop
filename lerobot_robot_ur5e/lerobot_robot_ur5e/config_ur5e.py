from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from lerobot.robots.config import RobotConfig

@RobotConfig.register_subclass("ur5e_robot")
@dataclass
class UR5eConfig(RobotConfig):
    use_gripper: bool = True
    init_gripper: bool = True
    gripper_reverse: bool = False
    robot_ip: str = "192.168.1.184"
    gripper_port: str = "/dev/ur5e_left_gripper"
    gripper_force: int = 70
    gripper_speed: int = 60
    gripper_bin_threshold: float = 0.98
    close_threshold: float = 0.7
    debug: bool = False
    kp: int = 1200
    kd: int = 600
    kp_rot: int = 2000
    kd_rot: int = 400
    hold_current_pose_on_idle: bool = False
    rtde_freq: int = 100
    select_vector: list = field(default_factory=lambda: [1, 1, 1, 0, 0, 0])
    force_limit: list = field(default_factory=lambda: [2, 2, 2, 2, 2, 2])
    pos_delta: float = 0.02
    vel_delta: float = 4.0
    gain_scale: float = 1.5
    speed: float = 0.5
    acceleration: float = 0.5
    servo_time: float = 0.1
    lookahead_time: float = 0.1
    gain: int = 300
    payload_mass: float = 1.601
    payload_cog: list = field(default_factory=lambda: [0.011, -0.002, 0.052])
    init_pose: list = field(default_factory=lambda: [0.614137, -0.074413, 0.122977, 3.106029, -0.009024, -1.636309])
    init_pose_range: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    reference_frame: str = "base"  # "base" or "tcp"
    control_space: str = "position"  # "position" or "force"
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
