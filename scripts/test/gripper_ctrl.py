from pathlib import Path

import yaml
from pyDHgripper import PGE


def get_vel(gripper):
    return gripper.write_uart(
        modbus_high_addr=0x01,
        modbus_low_addr=0x04,
        is_set=False,
    )


def load_gripper_config() -> tuple[str, int, int]:
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)
    robot_cfg = cfg["record"]["robot"]
    return (
        robot_cfg["gripper_port"],
        robot_cfg.get("gripper_force", 70),
        robot_cfg.get("gripper_speed", 60),
    )


def main():
    gripper_port, gripper_force, gripper_speed = load_gripper_config()
    print(f"Connecting to gripper at {gripper_port}...")

    gripper = PGE(gripper_port)
    gripper.init_feedback()
    gripper.set_force(gripper_force)
    gripper.set_vel(gripper_speed)
    print(f"Configured gripper force: {gripper_force}")
    print(f"Configured gripper speed: {gripper_speed}")
    print(f"Gripper velocity: {get_vel(gripper)}")
    print("Enter a target position from 0 to 1000. Press Ctrl+C to exit.")

    while True:
        val = input("Target position: ").strip()
        gripper.set_pos(val=int(val), blocking=False)


if __name__ == "__main__":
    main()
