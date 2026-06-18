import logging
import time
from pathlib import Path
from typing import Any

import yaml
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation as R


logging.basicConfig(level=logging.INFO, format="%(message)s")


def load_robot_ip() -> str:
    parent_path = Path(__file__).resolve().parent
    config_path = parent_path.parent / "config" / "cfg.yaml"
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["record"]["robot"]["ip"]


def format_vector(values: list[float] | tuple[float, ...], precision: int = 6) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


def tcp_pose_rotvec_to_euler(pose: list[float] | tuple[float, ...]) -> list[float]:
    return [
        *pose[:3],
        *R.from_rotvec(pose[3:]).as_euler("xyz").tolist(),
    ]


def read_robot_state(rtde_r: RTDEReceiveInterface, rtde_c: RTDEControlInterface) -> dict[str, Any]:
    tcp_pose = rtde_r.getActualTCPPose()
    return {
        "joint_positions_rad": rtde_r.getActualQ(),
        "joint_velocities_rad_s": rtde_r.getActualQd(),
        "joint_accelerations_rad_s2": rtde_r.getTargetQdd(),
        "joint_torques_nm": rtde_c.getJointTorques(),
        "tcp_pose_m_rad": tcp_pose,
        "tcp_pose_euler_m_rad": tcp_pose_rotvec_to_euler(tcp_pose),
        "tcp_offset_m_rad": rtde_c.getTCPOffset(),
        "tcp_speed_m_s_rad_s": rtde_r.getActualTCPSpeed(),
        "tcp_acceleration_m_s2": rtde_r.getActualToolAccelerometer(),
        "tcp_force_n_nm": rtde_r.getActualTCPForce(),
    }


def print_robot_state(state: dict[str, Any]) -> None:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    logging.info("")
    logging.info(f"Robot state at {timestamp}")
    logging.info(f"  Joint positions [rad]:        {format_vector(state['joint_positions_rad'])}")
    logging.info(f"  Joint velocities [rad/s]:     {format_vector(state['joint_velocities_rad_s'])}")
    logging.info(f"  Joint accelerations [rad/s^2]: {format_vector(state['joint_accelerations_rad_s2'])}")
    logging.info(f"  Joint torques [Nm]:           {format_vector(state['joint_torques_nm'])}")
    logging.info(f"  TCP pose rotvec [m, rad]:     {format_vector(state['tcp_pose_m_rad'])}")
    logging.info(f"  TCP pose euler XYZ [m, rad]:  {format_vector(state['tcp_pose_euler_m_rad'])}")
    logging.info(f"  TCP offset [m, rad]:          {format_vector(state['tcp_offset_m_rad'])}")
    logging.info(f"  TCP speed [m/s, rad/s]:       {format_vector(state['tcp_speed_m_s_rad_s'])}")
    logging.info(f"  TCP acceleration [m/s^2]:     {format_vector(state['tcp_acceleration_m_s2'])}")
    logging.info(f"  TCP force [N, Nm]:            {format_vector(state['tcp_force_n_nm'])}")


def main() -> None:
    robot_ip = load_robot_ip()
    logging.info(f"Connecting to UR5e robot at {robot_ip}...")
    rtde_r = RTDEReceiveInterface(robot_ip)
    rtde_c = RTDEControlInterface(robot_ip)
    logging.info("Connected.")

    try:
        print_robot_state(read_robot_state(rtde_r, rtde_c))
    finally:
        rtde_c.disconnect()
        rtde_r.disconnect()
        logging.info("Disconnected from robot.")


if __name__ == "__main__":
    main()
