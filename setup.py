from setuptools import find_packages, setup


setup(
    name="lerobot_ur5e_keyteleop",
    version="0.1.0",
    description="UR5e keyboard teleoperation and dataset collection utilities",
    python_requires=">=3.10",
    packages=[
        *find_packages(where="lerobot_robot_ur5e"),
        *find_packages(where="lerobot_teleoperator_ur5e"),
        *find_packages(where=".", include=["scripts", "scripts.*"]),
    ],
    package_dir={
        "lerobot_robot_ur5e": "lerobot_robot_ur5e/lerobot_robot_ur5e",
        "lerobot_teleoperator_ur5e": "lerobot_teleoperator_ur5e/lerobot_teleoperator_ur5e",
    },
    include_package_data=True,
    install_requires=[
        "numpy",
        "pydhgripper",
        "pyrealsense2",
        "send2trash",
        "scipy",
        "ur-rtde",
    ],
    scripts=[
        "scripts/tools/map_gripper.sh",
    ],
    entry_points={
        "console_scripts": [
            "ur5e-record = scripts.core.run_record:main",
            "ur5e-replay = scripts.core.run_replay:main",
            "ur5e-visualize = scripts.core.run_visualize:main",
            "tools-check-rs = scripts.tools.rs_devices:list_realsense_devices",
            "tools-check-info = scripts.tools.check_dataset_info:main",
            "tools-check-dataset = scripts.tools.check_dataset:main",
            "tools-prune-dataset = scripts.tools.prune_episodes:main",
            "tools-robot-state = scripts.tools.read_robot_state:main",
            "test-gripper-ctrl = scripts.test.gripper_ctrl:main",
            "ur5e-help = scripts.help.help_info:main",
        ],
    },
)
