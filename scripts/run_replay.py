import time
import yaml
from pathlib import Path
from typing import Dict, Any
from lerobot_robot_ur5e import UR5eConfig, UR5e
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import log_say

class ReplayConfig:
    def __init__(self, cfg: Dict[str, Any]):
        robot = cfg["robot"]

        # global config
        self.repo_id: str = cfg["repo_id"]
        self.episode_idx: str = cfg.get("episode_idx", 0)

        # robot config
        self.robot_ip: str = robot["ip"]
        self.gripper_port: str = robot["gripper_port"]

def main(replay_cfg: ReplayConfig):
    episode_idx = replay_cfg.episode_idx

    robot_config = UR5eConfig(
        robot_ip=replay_cfg.robot_ip,
        gripper_port=replay_cfg.gripper_port,
    )

    robot = robot = UR5e(robot_config)
    robot.connect()
    dataset = LeRobotDataset(replay_cfg.repo_id, episodes=[episode_idx])
    actions = dataset.hf_dataset.select_columns("action")
    log_say(f"Replaying episode {episode_idx}")
    for idx in range(dataset.num_frames):
        t0 = time.perf_counter()

        action = {
            name: float(actions[idx]["action"][i]) for i, name in enumerate(dataset.features["action"]["names"])
        }
        robot.send_action(action)

        busy_wait(1.0 / dataset.fps - (time.perf_counter() - t0))

    robot.disconnect()

if __name__ == "__main__":
    with open(Path(__file__).parent / "config" / "cfg.yaml", 'r') as f:
        cfg = yaml.safe_load(f)

    replay_cfg = ReplayConfig(cfg["replay"])

    main(replay_cfg)