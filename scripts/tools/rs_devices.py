"""
list_realsense_devices.py

扫描系统中连接的 Intel RealSense 相机，打印数量和序列号。
"""
import pyrealsense2 as rs

def list_realsense_devices():
    ctx = rs.context()
    devices = ctx.devices
    num_devices = len(devices)
    print(f"------------Detected {num_devices} RealSense device------------")

    if num_devices == 0:
        return

    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)
        print(f"Device {i}: Name={name}, Serial={serial}")

if __name__ == "__main__":
    list_realsense_devices()
