# UR5e 数据采集与回放指南

## 1. 环境配置

### 1.1 创建并激活 conda 环境
```bash
conda create -n ur_data python=3.10
conda activate ur_data
```

### 1.2 安装 lerobot
```bash
# 安装指定版本 0.3.4
# git checkout da5d2f3e9187fa4690e6667fe8b294cae49016d6
git clone https://github.com/huggingface/lerobot.git
pip install -e lerobot
```

### 1.3 克隆并安装 UR5e 遥操作控制源码
```bash
mkdir ur_data_collection && cd ur_data_collection
git clone https://github.com/scy-v/lerobot_ur5e_keyteleop.git
cd lerobot_ur5e_keyboard
pip install -e lerobot_robot_ur5e
pip install -e lerobot_teleoperator_ur5e
```

## 2. 获取和配置必要参数

### 2.1 获取 RealSense 相机序列号
```bash
python scripts/rs_devices.py
```

### 2.2 固定夹爪串口映射
例如，将夹爪映射到 `/dev/ur5e_left_gripper`：
```bash
bash scripts/map_gripper.sh <gripper_port_name>
```

### 2.3 配置其他参数
打开 `config/cfg.yaml` 文件，查看并修改所有必要参数，确保后续步骤正常运行。

## 3. 数据集记录

### 3.1 上传数据到 Hugging Face（可选）
1. 在 `cfg.yaml` 中设置：
```yaml
push_to_hub: True
```
2. 获取 Hugging Face 账号的 token 并登录：
```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} 
huggingface-cli whoami  # 检查是否登录成功
```

### 3.2 开始记录
1. 打开 UR5e 示教器上的 **Remote Control**。
2. 确认 `cfg.yaml` 中参数配置正确。
3. 运行以下命令开始记录数据：
```bash
python scripts/run_record.py
```

## 4. 数据集回放
```bash
python scripts/run_replay.py #注意cfg配置
```

## 5. 数据集可视化
```bash
python scripts/run_visualize.py #注意cfg配置
```

## 6. 数据集追加与恢复
如果你已经在指定的 `repo_id` 下录制过数据集，可以在 `cfg.yaml` 中将 `resume` 设置为 `True`，以在现有数据集的基础上继续录制新的数据。然后运行以下命令：
```bash
python scripts/run_record.py
```

## 7. 数据集合并
如果你在不同阶段分别录制了数据集，请确保各阶段的数据集具有不同的 `repo_id`, 完成录制后，可通过以下命令将它们合并为一个数据集: 
```bash
lerobot-edit-dataset 
    --repo_id <merged_repo_id> 
    --operation.type merge 
    --operation.repo_ids "['<repo_id_1>', '<repo_id2>']"
```

## 8. 录制控制按键说明
1. **右方向键**  
   - 按下后：结束当前 episode 的录制并保存数据
   - 系统会自动进入 **复位（reset）** 阶段，请通过主臂将从臂归位初始位姿
   - 再次按下：开始录制下一个 episode

2. **左方向键**  
   - 按下后：重新开始当前 episode 的录制

3. **Esc 键**  
   - 按下后：退出整个录制任务，并保存当前已录制的数据