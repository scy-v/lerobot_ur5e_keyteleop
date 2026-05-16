from pathlib import Path

import yaml
from pyDHgripper import PGE


def get_vel(gripper):
    return gripper.write_uart(
        modbus_high_addr=0x01,
        modbus_low_addr=0x04,
        is_set=False,
    )


def load_gripper_port() -> str:
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["record"]["robot"]["gripper_port"]


def main():
    gripper_port = load_gripper_port()
    print(f"Connecting to gripper at {gripper_port}...")

    gripper = PGE(gripper_port)
    gripper.init_feedback()
    gripper.set_force(100)
    gripper.set_vel(100)
    print(f"Gripper velocity: {get_vel(gripper)}")
    print("Enter a target position from 0 to 1000. Press Ctrl+C to exit.")

    while True:
        val = input("Target position: ").strip()
        gripper.set_pos(val=int(val), blocking=True)


if __name__ == "__main__":
    main()
