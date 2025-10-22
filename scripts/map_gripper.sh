#!/bin/bash

# 用法: sudo ./map_ur5e_usb.sh ur5e_left_gripper
# 参数 1：映射名称
MAP_NAME=$1

if [ -z "$MAP_NAME" ]; then
    echo "Usage: $0 <mapping_name>"
    exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-ur5e.rules"

# 1. 获取当前 ttyUSB 设备数量
USB_DEVICES=($(ls /dev/ttyUSB* 2>/dev/null))
NUM_USB=${#USB_DEVICES[@]}

if [ "$NUM_USB" -eq 0 ]; then
    echo "没有检测到任何 /dev/ttyUSB* 设备"
    exit 1
elif [ "$NUM_USB" -gt 1 ]; then
    echo "此映射 USB 设备只能插入一个，现在有 $NUM_USB 个"
    exit 1
fi

DEVICE=${USB_DEVICES[0]}

# 2. 获取设备的 idVendor, idProduct, serial
IDVENDOR=$(udevadm info -a -n $DEVICE | grep 'ATTRS{idVendor}' | head -n1 | awk -F'"' '{print $2}')
IDPRODUCT=$(udevadm info -a -n $DEVICE | grep 'ATTRS{idProduct}' | head -n1 | awk -F'"' '{print $2}')
SERIAL=$(udevadm info -a -n $DEVICE | grep 'ATTRS{serial}' | head -n1 | awk -F'"' '{print $2}')

if [ -z "$SERIAL" ]; then
    echo "无法获取设备序列号"
    exit 1
fi

# 3. 检测规则文件是否存在
if [ ! -f "$RULE_FILE" ]; then
    sudo touch "$RULE_FILE"
fi

# 4. 检查映射名是否已经存在
EXISTING_LINE=$(grep "SYMLINK+=\"$MAP_NAME\"" $RULE_FILE)

if [ -n "$EXISTING_LINE" ]; then
    EXISTING_SERIAL=$(echo $EXISTING_LINE | grep -o 'ATTRS{serial}=="[^"]*"' | awk -F'"' '{print $2}')
    if [ "$EXISTING_SERIAL" == "$SERIAL" ]; then
        echo "规则已存在且序列号相同，无需修改"
    else
        echo "规则已存在但序列号不同，替换为新的"
        sudo sed -i "/SYMLINK+=\"$MAP_NAME\"/d" $RULE_FILE
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$MAP_NAME\"" | sudo tee -a $RULE_FILE
    fi
else
    echo "添加新规则..."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$MAP_NAME\"" | sudo tee -a $RULE_FILE
fi

# 5. 重新加载 udev 规则
sudo udevadm control --reload
sudo udevadm trigger
sleep 2

# 6. 检查映射是否生效
if [ -e "/dev/$MAP_NAME" ]; then
    LINK_TARGET=$(readlink -f "/dev/$MAP_NAME")
    echo "映射检查成功: /dev/$MAP_NAME -> $LINK_TARGET"
    # 可选：显示序列号
    SERIAL_CHECK=$(udevadm info -a -n "$LINK_TARGET" | grep 'ATTRS{serial}' | head -n1 | awk -F'"' '{print $2}')
    echo "   对应序列号: $SERIAL_CHECK"
else
    echo "映射失败: /dev/$MAP_NAME 不存在"
fi
