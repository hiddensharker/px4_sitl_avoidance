"""
阶段一 Step 1 — AirSim Python API 连接 + 读取 LiDAR 点云
依赖: pip install airsim msgpack-rpc-python numpy
运行前确保 AirSim + PX4 SITL 已启动
"""

import time
import numpy as np
import airsim

# ──────────────────────────────────────────
# 1. 连接 AirSim
# ──────────────────────────────────────────
client = airsim.MultirotorClient(ip="172.24.112.1", port=41451)
client.confirmConnection()
print("[OK] AirSim 连接成功")

# ──────────────────────────────────────────
# 2. LiDAR 配置说明（settings.json 需提前写好）
# LiDAR 名称默认 "Lidar"，可按需修改
# ──────────────────────────────────────────
LIDAR_NAME = "Lidar"
VEHICLE_NAME = ""          # 单机留空即可

# ──────────────────────────────────────────
# 3. 持续读取点云并打印
# ──────────────────────────────────────────
print(f"[INFO] 开始读取 LiDAR: {LIDAR_NAME}，Ctrl+C 退出\n")

try:
    while True:
        data = client.getLidarData(lidar_name=LIDAR_NAME, vehicle_name=VEHICLE_NAME)

        if len(data.point_cloud) < 3:
            print("[WARN] 点云为空（飞机可能未起飞，或 LiDAR 未配置）")
        else:
            # point_cloud 是展平的 [x,y,z, x,y,z, ...] float 列表
            pts = np.array(data.point_cloud, dtype=np.float32).reshape(-1, 3)
            print(f"[点云] 帧时间戳={data.time_stamp}  点数={len(pts)}")
            print(f"       X range: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}]")
            print(f"       Y range: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}]")
            print(f"       Z range: [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}]")
            print(f"       前5点(机体系):\n{pts[:5]}\n")

        time.sleep(0.5)   # 2 Hz 采样，够验证用

except KeyboardInterrupt:
    print("\n[STOP] 退出")