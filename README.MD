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
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .
```

### 1.3 Clone and Install UR5e Teleoperation Control Source
```bash
mkdir ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_keyteleop.git
cd lerobot_ur5e_keyteleop
pip install -e lerobot_robot_ur5e
pip install -e lerobot_teleoperator_ur5e
```

## 2. Obtain and Configure Necessary Parameters

### 2.1 Get RealSense Camera Serial Number
```bash
bash scripts/rs_devices.py
```

### 2.2 Map Gripper Serial Port
For example, map the gripper to `"/dev/ur5e_left_gripper"`:
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
