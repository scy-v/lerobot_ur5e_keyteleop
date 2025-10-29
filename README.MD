# UR5e Data Collection and Playback Guide

## 1. Environment Setup

### 1.1 Create and Activate Conda Environment
```bash
conda create -n ur_data python=3.10
conda activate ur_data
```

### 1.2 Install lerobot
```bash
# Install specific version 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
pip install -e lerobot
```

### 1.3 Clone and Install UR5e Teleoperation Control Source
```bash
mkdir ur_data_collection && cd ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_keyteleop.git
cd lerobot_ur5e_keyteleop
pip install -e lerobot_robot_ur5e
pip install -e lerobot_teleoperator_ur5e
```

## 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
```bash
python scripts/rs_devices.py
```

### 2.2 Map Gripper Serial Port
For example, map the gripper to `/dev/ur5e_left_gripper`:
```bash
bash scripts/map_gripper.sh <gripper_port_name>
```

### 2.3 Configure Other Parameters
Open the `config/cfg.yaml` file and review or modify all required parameters to ensure subsequent steps run correctly.

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
3. Run the following command to start recording data:
```bash
python scripts/run_record.py
```

## 4. Dataset Playback
```bash
python scripts/run_replay.py # Make sure cfg.yaml is properly configured
```

## 5. Dataset Visualization
```bash
python scripts/run_visualize.py # Make sure cfg.yaml is properly configured
```

## 6. Dataset Appending and Resume
If you already recorded an old dataset with `repo_id`, set `resume: True` in `cfg.yaml`to append recording new data to the same dataset. Then, run the following command:
```bash
python scripts/run_record.py
```

## 7. Merge Datasets
If you recorded data in multiple stages, ensure each dataset has a unique `repo_id`. Use the following command to merge them into a single dataset:
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```

## 8. Recording Control Keys

1. **Right Arrow Key**  
   - Press once to finish the current episode and save the data.  
   - The system enters **reset** mode automatically. Move the slave arm to the start pose using the master arm.  
   - Press again to start recording the next episode.

2. **Left Arrow Key**  
   - Press to restart the current episode recording (overwrite the current data).

3. **Esc Key**  
   - Press to exit the recording session and save all recorded episodes.