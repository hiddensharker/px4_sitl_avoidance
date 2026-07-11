#!/usr/bin/env python3
"""
airsim_lidar.py — AirSim LiDAR 点云读取封装
依赖: pip install airsim msgpack-rpc-python numpy
"""

import numpy as np
import airsim


class LidarReader:
    """
    封装 AirSim LiDAR 点云读取。
    使用方式:
        lidar = LidarReader()
        pts = lidar.get_points()   # np.ndarray, shape (N, 3), NED 机体系
    """

    def __init__(
        self,
        ip: str = "172.24.112.1",
        port: int = 41451,
        lidar_name: str = "Lidar",
        vehicle_name: str = "",
    ):
        self.lidar_name = lidar_name
        self.vehicle_name = vehicle_name

        print(f"[LiDAR] 连接 AirSim {ip}:{port} ...")
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        print("[LiDAR] 连接成功")

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------

    def get_points(self) -> np.ndarray | None:
        """
        读取一帧点云。
        返回 np.ndarray shape (N, 3)，坐标系为 AirSim NED 机体系 (x前 y右 z下)。
        若点云为空返回 None。
        """
        data = self.client.getLidarData(
            lidar_name=self.lidar_name,
            vehicle_name=self.vehicle_name,
        )

        if len(data.point_cloud) < 3:
            return None

        pts = np.array(data.point_cloud, dtype=np.float32).reshape(-1, 3)
        return pts

    def get_points_with_timestamp(self) -> tuple[int, np.ndarray | None]:
        """
        返回 (timestamp, points)，timestamp 单位为纳秒。
        """
        data = self.client.getLidarData(
            lidar_name=self.lidar_name,
            vehicle_name=self.vehicle_name,
        )

        if len(data.point_cloud) < 3:
            return data.time_stamp, None

        pts = np.array(data.point_cloud, dtype=np.float32).reshape(-1, 3)
        return data.time_stamp, pts

    def print_summary(self, pts: np.ndarray):
        """调试用：打印点云统计信息"""
        if pts is None:
            print("[LiDAR] 点云为空")
            return
        print(f"[LiDAR] 点数={len(pts)}")
        print(f"        X: [{pts[:, 0].min():.2f}, {pts[:, 0].max():.2f}]")
        print(f"        Y: [{pts[:, 1].min():.2f}, {pts[:, 1].max():.2f}]")
        print(f"        Z: [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]")


# ------------------------------------------------------------------
# 独立运行测试
# ------------------------------------------------------------------
if __name__ == "__main__":
    import time

    lidar = LidarReader()
    print("[INFO] 开始读取点云，Ctrl+C 退出\n")
    try:
        while True:
            ts, pts = lidar.get_points_with_timestamp()
            print(f"[帧] ts={ts}")
            lidar.print_summary(pts)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[STOP] 退出")