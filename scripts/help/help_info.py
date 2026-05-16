def main():
    print("""
==================================================
 UR5e Keyboard Teleoperation Utilities
==================================================

Core Commands:
  ur5e-record       Record keyboard teleoperation dataset
  ur5e-replay       Replay a recorded dataset
  ur5e-visualize    Visualize recorded dataset

Tool Commands:
  tools-check-dataset   Check local dataset integrity
  tools-check-info      Check local dataset information
  tools-check-rs        Retrieve connected RealSense camera serial numbers
  tools-prune-dataset   Prune episodes from dataset by Episode ID

Shell Tools:
  map_gripper.sh    Map Gripper Serial Port

--------------------------------------------------
 Tip: Use 'ur5e-help' anytime to see this summary.
==================================================
""")
