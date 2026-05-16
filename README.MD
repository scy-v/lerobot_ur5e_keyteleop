# UR5e/UR7e Keyboard Teleoperation Data Collection

<p align="center">
  <img src="assets/robot.png" alt="Robot" width="600">
  <br>
</p>

## 🚀 Teleoperation Platform Updates

<div align="center">

| Repository | Description | GitHub Link |
|:----------:|:------------|:-----------:|
| **lerobot_ur5e_isoteleop** | Master-slave UR5e/UR7e teleoperation with joint-level isomorphic mapping, built on LeRobot. | [GitHub](https://github.com/scy-v/lerobot_ur5e_isoteleop) |
| **lerobot_ur_dual_vrteleop** | Dual-arm VR teleoperation for UR5e/UR7e robots, built on LeRobot and PICO 4 Ultra. | [GitHub](https://github.com/scy-v/lerobot_ur_dual_vrteleop) |

</div>

## 1. Environment Setup

### 1.1 Create and Activate Conda Environment
```bash
conda create -n ur_data python=3.10
conda activate ur_data
```

### 1.2 Install lerobot
```bash
# Install specific version 0.3.4
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
pip install -e .
```

### 1.3 Clone and Install UR5e Keyboard Teleoperation Source
```bash
mkdir ur_data_collection && cd ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_keyteleop.git
cd lerobot_ur5e_keyteleop
pip install -r requirements.txt # install all packages in editable mode
```

### 1.4 View Available Commands
Run the following command to display all available scripts:
```bash
ur5e-help
# ==================================================
#  UR5e Keyboard Teleoperation Utilities
# ==================================================
#
# Core Commands:
#   ur5e-record       Record keyboard teleoperation dataset
#   ur5e-replay       Replay a recorded dataset
#   ur5e-visualize    Visualize recorded dataset
#
# Tool Commands:
#   tools-check-dataset   Check local dataset integrity
#   tools-check-info      Check local dataset information
#   tools-check-rs        Retrieve connected RealSense camera serial numbers
#   tools-prune-dataset   Prune episodes from dataset by Episode ID
#   tools-robot-state     Read and print robot state once
#
# Shell Tools:
#   map_gripper.sh        Map Gripper Serial Port
#
# Test Commands:
#   test-gripper-ctrl     Run gripper control command
#
# --------------------------------------------------
#  Tip: Use 'ur5e-help' anytime to see this summary.
# ==================================================
```

## 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
Please make sure only one camera is connected at a time.
```bash
tools-check-rs
```

### 2.2 Map Gripper Serial Port
For example, map the gripper to `/dev/ur5e_left_gripper` and make sure only one gripper USB device is connected:
```bash
map_gripper.sh ur5e_left_gripper
```
Next, enter the mapped device into the `gripper_port` field in `cfg.yaml`.

### 2.3 Configure Keyboard Teleoperation and Robot Parameters
Open `scripts/config/cfg.yaml` and check the robot IP, gripper port, camera serial numbers, control mode, reset pose, payload, and keyboard step sizes.

Key robot fields:
- `control_space`: `"position"` or `"force"`.
- `reference_frame`: `"base"` or `"tcp"`.
  - `base`: action delta is expressed in the base frame, and `tcp_pose` observation is absolute.
  - `tcp`: action delta is expressed in the current end-effector frame, and `tcp_pose` observation is relative to the episode reference pose.
  - For consistency across UR TCP-offset changes, `tcp_pose` stores the EE/flange pose converted from UR TCP pose and TCP offset.
- `init_pose`: reset TCP pose `[x, y, z, rx, ry, rz]`.
- `init_pos_range`: random reset position range for `[x, y, z]`.
- `teleop.position_step_size`: keyboard translation delta for position control.
- `teleop.force_step_size`: keyboard translation delta for force control.
- `teleop.position_rot_step_size`: keyboard Euler-angle delta for position control.
- `teleop.force_rot_step_size`: keyboard Euler-angle delta for force control.

## 3. Dataset Recording

### 3.1 Upload Data to Hugging Face (Optional)
1. Set the following in `cfg.yaml`:
```yaml
push_to_hub: True
```
2. Obtain your Hugging Face account token and login:
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN}
huggingface-cli whoami  # Verify successful login
```

### 3.2 Start Recording
1. Turn on **Remote Control** on the UR5e teach pendant.
2. Verify that the parameters in `cfg.yaml` are correct.
3. To ensure standardized data collection, please read `10. Dataset Naming and Recording Details` before running the following command:
```bash
ur5e-record
```

<p align="center">
  <img src="assets/record.png" alt="Record" width="600">
  <br>
  <b>Figure 1: Record</b>
</p>

## 4. Dataset Replay
```bash
ur5e-replay # Make sure cfg.yaml is properly configured
```

## 5. Dataset Visualization
```bash
ur5e-visualize # Make sure cfg.yaml is properly configured
```

| Visualization | Episode Data |
|:-------------:|:------------:|
| <img src="assets/visualize.png" alt="Visualization" width="460"> | <img src="assets/episode_data.png" alt="Episode Data" width="460"> |
| **Figure 2: Visualization** | **Figure 3: Episode Data Window** |

The data window shows the action for each frame and lets you select one observation group at a time, such as `tcp_pose`, `tcp_force`, or joint states.

## 6. Dataset Appending and Resume
If you have already recorded a dataset under the specified `repo_id`, set `resume: True` in `cfg.yaml` and enter the dataset name in `resume_dataset` to continue recording new data on the existing dataset. Then, run the following command:

```bash
ur5e-record
```

## 7. Merge Datasets
If you recorded data in multiple stages, ensure each dataset has a unique `repo_id`. Use the following command to merge them into a single dataset:
```bash
lerobot-edit-dataset \
    --repo_id <merged_repo_id> \
    --operation.type merge \
    --operation.repo_ids "['<repo_id_1>', '<repo_id_2>']"
```
- For more dataset processing commands, see [LeRobot](https://huggingface.co/docs/lerobot/using_dataset_tools).

## 8. Dataset Check and Pruning
- Run the following command to verify dataset integrity, especially for missing video frames.
- Note: update `check_dataset.dataset_name` in `cfg.yaml`.
```bash
tools-check-dataset
```
- If missing frames are detected, create a new dataset excluding the affected episode IDs while keeping the original dataset intact.
- Note: update `prune_episodes.old_dataset_name` and `prune_episodes.new_dataset_name` in `cfg.yaml`.
```bash
tools-prune-dataset "[id1,...,idN]"
```
- This tool can remove episodes with missing frames or any specified episodes.

## 9. Recording Control Keys

### 9.1 Episode Control Keys
1. **Right Arrow Key**
   1. Press the right arrow key: end the current episode and save the data. The program will pause.
   2. Press Enter to enter the **reset** phase. Use keyboard teleoperation to move the robot back to a safe pre-reset pose.
   3. Press the right arrow key to finish manual reset control. The robot will then automatically reset to the configured initial TCP pose.
   4. Press Enter to start recording the next episode.

2. **Left Arrow Key**
   - Press to restart the current episode recording and overwrite the current episode buffer.

3. **Esc Key**
   - Press to exit the recording session and save all recorded episodes.

4. **Ctrl+C or Exception**
   - Pressing Ctrl+C or encountering an exception enters exception handling and asks whether to delete the incomplete dataset.

### 9.2 Keyboard Teleoperation Keys
| Key | Action |
|:---:|:-------|
| `w` / `s` | Move along x negative / positive |
| `a` / `d` | Move along y negative / positive |
| `q` / `e` | Move along z positive / negative |
| `r` / `t` | Rotate rx positive / negative |
| `g` / `f` | Rotate ry positive / negative |
| `b` / `v` | Rotate rz positive / negative |
| `o` | Latch gripper open |
| `l` | Latch gripper close |

## 10. Dataset Naming and Recording Details

### 10.1 Dataset Naming

<p align="center">
  <img src="assets/dataset.png" alt="dataset" width="600">
  <br>
  <b>Figure 4: Dataset</b>
</p>

<p align="center">
  <img src="assets/dataset_info.png" alt="dataset_info" width="600">
  <br>
  <b>Figure 5: Dataset Info</b>
</p>

1. The dataset is by default stored in the `~/.cache/huggingface/lerobot` directory and contains three types of items:

   - `dataset_info.txt`: Automatically records local dataset information, including the following fields: `record_id`, `name`, `task`, `date`, `version`, `user_info`, and `type`. The `user_info` field can be annotated via the `user_notes` entry in `cfg.yaml`.

   - `dataset_info_backup`: When `tools-check-dataset` is used to manually update `dataset_info.txt`, this folder stores backups of the previous records.

   - Dataset folders: Stores the actual dataset contents.

2. Dataset names follow the format `[description]_[date]_[version]`, where:
   - `description` comes from `cfg.yaml` as `repo_id=<user_name>/<description>`;
   - `date` is generated automatically;
   - `version` is incremented automatically if a dataset with the same `repo_id` already exists.

3. Description naming rule: `task.description -> Verb_SourceObject_prep_TargetObject`.
   For example:
   `Pick up the green cube and put it into the trash bin -> pick_greencube_into_trashbin`.

### 10.2 Recording Instructions
1. Complete step `2. Obtain and Configure Necessary Parameters`.
2. Fill in the task instruction in `cfg.yaml` under `task.description` and set `repo_id` according to the dataset naming rules.
3. Check and adjust other parameters in `cfg.yaml` to ensure correctness.
4. Run `ur5e-record`. The robot will reset to `init_pose` before recording, and each episode will set a new episode reference pose at the start of recording.
5. After recording, a `dataset_info.txt` file will be automatically created in the same directory to store local dataset information. If you have manually deleted any datasets, update `dataset_info.txt` using:
```bash
tools-check-info
```
