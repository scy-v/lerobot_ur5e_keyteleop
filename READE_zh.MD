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
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e .
```

### 1.3 克隆并安装 UR5e 遥操作控制源码
```bash
git clone ...git
cd lerobot_ur5e_keyboard
pip install -e lerobot_robot_ur5e
pip install -e lerobot_teleoperator_ur5e
```

## 2. 获取和配置必要参数

### 2.1 获取 RealSense 相机序列号
```bash
bash scripts/rs_devices.py
```

### 2.2 固定夹爪串口映射
例如，将夹爪映射到 `"/dev/ur5e_left_gripper"`：
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
