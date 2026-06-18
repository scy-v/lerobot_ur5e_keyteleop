from pathlib import Path

import crcmod
import serial
import yaml
from pyDHgripper import PGE


def get_vel(gripper):
    return gripper.write_uart(
        modbus_high_addr=0x01,
        modbus_low_addr=0x04,
        is_set=False,
    )


def load_gripper_config() -> tuple[str, bool, int, int]:
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with open(cfg_path, "r") as f:
        cfg = yaml.safe_load(f)
    robot_cfg = cfg["record"]["robot"]
    return (
        robot_cfg["gripper_port"],
        robot_cfg.get("init_gripper", True),
        robot_cfg.get("gripper_force", 70),
        robot_cfg.get("gripper_speed", 60),
    )


def connect_gripper(port: str, init_gripper: bool):
    if init_gripper:
        gripper = PGE(port)
        gripper.init_feedback()
        return gripper

    gripper = PGE.__new__(PGE)
    gripper.ser = serial.Serial(port=port, baudrate=115200)
    gripper.crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    print("Skipped init_state and init_feedback by config.")
    return gripper


def main():
    gripper_port, init_gripper, gripper_force, gripper_speed = load_gripper_config()
    print(f"Connecting to gripper at {gripper_port}...")

    gripper = connect_gripper(gripper_port, init_gripper)
    gripper.set_force(gripper_force)
    gripper.set_vel(gripper_speed)
    print(f"Init gripper: {init_gripper}")
    print(f"Configured gripper force: {gripper_force}")
    print(f"Configured gripper speed: {gripper_speed}")
    print(f"Gripper velocity: {get_vel(gripper)}")
    print("Enter a target position from 0 to 1000. Press Ctrl+C to exit.")

    while True:
        val = input("Target position: ").strip()
        gripper.set_pos(val=int(val), blocking=False)


if __name__ == "__main__":
    main()
