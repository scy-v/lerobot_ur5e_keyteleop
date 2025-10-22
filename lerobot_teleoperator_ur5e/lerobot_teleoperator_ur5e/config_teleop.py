from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("lerobot_teleoperator_ur5e")
@dataclass
class UR5eTeleopConfig(TeleoperatorConfig):
    port: str = "4443"
    host: str = "0.0.0.0"
    use_gripper: bool = True
