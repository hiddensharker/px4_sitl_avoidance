#!/usr/bin/env python3
"""
vfh.py — 三维 VFH 球面直方图
子任务二：点云 → 球面直方图构建
子任务三：VFH 最优安全方向选取 + 速度指令计算

坐标系约定（机体系）:
    X 正: 机头前方
    Y 正: 机体右侧
    Z 正: 机体正下方

球面分格:
    方位角 (azimuth)  : X-Y 平面内，从 X 轴正方向逆时针，范围 (-180°, 180°]，分辨率 10°，共 36 格
    仰角   (elevation): 从 X-Y 平面向上为正，向下为负，范围 [-90°, 90°]，分辨率 10°，共 18 格

格子存储值:
    该方向上最近点的距离（m），无点的格子填 np.inf
"""

import math
import numpy as np


# ======================================================================
# 常量
# ======================================================================
AZ_RES_DEG     = 10                       # 方位角分辨率（度）
EL_RES_DEG     = 10                       # 仰角分辨率（度）
AZ_BINS        = 360 // AZ_RES_DEG       # 36
EL_BINS        = 180 // EL_RES_DEG       # 18
TRIGGER_DIST_M = 5.0                      # 触发避障的距离阈值（m）
EXCLUDE_DIST_M = 2.0                      # 方向选择时排除格子的距离阈值（m）
LIDAR_RANGE_M  = 10.0                     # LiDAR 最大量程（m）
MAX_SPEED_MPS  = 2                     # 避障最大速度（m/s）
NEIGHBOR_RING  = 1                        # 邻域圈数（1 = 3×3）


# ======================================================================
# VFHMap
# ======================================================================
class VFHMap:
    """
    球面直方图：每帧点云独立重建，每格存储最近点距离。

    典型用法:
        vfh = VFHMap()
        histogram = vfh.build(pts)          # np.ndarray shape (36, 18)
        triggered = vfh.is_obstacle_near()  # bool
    """

    def __init__(
        self,
        az_res_deg: float  = AZ_RES_DEG,
        el_res_deg: float  = EL_RES_DEG,
        trigger_dist_m: float = TRIGGER_DIST_M,
        exclude_dist_m: float = EXCLUDE_DIST_M,    # 新增
    ):
        self.az_res  = az_res_deg
        self.el_res  = el_res_deg
        self.az_bins = round(360 / az_res_deg)   # 36
        self.el_bins = round(180 / el_res_deg)   # 18
        self.trigger_dist = trigger_dist_m
        self.exclude_dist = exclude_dist_m          # 新增

        # 当前帧直方图，shape (az_bins, el_bins)，初始全 inf
        self.histogram: np.ndarray = np.full(
            (self.az_bins, self.el_bins), np.inf, dtype=np.float32
        )

    # ------------------------------------------------------------------
    # 公开接口
    # ------------------------------------------------------------------
    def build(self, pts: np.ndarray) -> np.ndarray:
        """
        输入一帧机体系点云 shape (N, 3)，重建球面直方图。
        返回 shape (az_bins, el_bins) 的数组，值为各格最近点距离（m）。
        无点格子值为 np.inf。
        """
        # 每帧独立重建：清空
        self.histogram[:] = np.inf

        if pts is None or len(pts) == 0:
            return self.histogram.copy()

        # ── 1. 直角坐标 → 球坐标 ──────────────────────────────
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

        dist = np.sqrt(x**2 + y**2 + z**2)

        # 过滤零距离点（传感器自身噪声）
        valid = dist > 0.0
        x, y, z, dist = x[valid], y[valid], z[valid], dist[valid]

        if len(dist) == 0:
            return self.histogram.copy()

        # 方位角：X-Y 平面内从 X 轴正方向逆时针，arctan2(y, x)，范围 (-π, π]
        az_rad = np.arctan2(y, x)

        # 仰角：Z 正向下，仰角定义为从 X-Y 平面向上为正
        # 即 elevation = -arcsin(z / dist)（z 正向下故取负）
        el_rad = -np.arcsin(np.clip(z / dist, -1.0, 1.0))

        # ── 2. 角度 → 格子索引 ────────────────────────────────
        az_deg = np.degrees(az_rad)   # (-180, 180]
        el_deg = np.degrees(el_rad)   # [-90, 90]

        # 方位角：(-180, 180] → [0, 360) → 格子索引 [0, az_bins)
        az_idx = (np.floor((az_deg + 180.0) / self.az_res) % self.az_bins).astype(np.int32)

        # 仰角：[-90, 90] → [0, 180) → 格子索引 [0, el_bins)
        el_idx = (np.floor((el_deg + 90.0) / self.el_res)).astype(np.int32)
        el_idx = np.clip(el_idx, 0, self.el_bins - 1)

        # ── 3. 更新最近距离 ────────────────────────────────────
        # 逐点更新（向量化：按格子分组取最小值）
        flat_idx = az_idx * self.el_bins + el_idx
        hist_flat = self.histogram.ravel()

        # np.minimum.at 实现逐格取最小值
        np.minimum.at(hist_flat, flat_idx, dist)

        return self.histogram.copy()

    def is_obstacle_near(self) -> bool:
        """
        判断当前帧是否有障碍物进入触发距离（任意方向）。
        需在 build() 之后调用。
        """
        return bool(np.any(self.histogram < self.trigger_dist))

    def nearest_obstacle_dist(self) -> float:
        """返回当前帧全向最近障碍距离（m），无障碍返回 inf"""
        return float(self.histogram[self.histogram < np.inf].min()) \
            if np.any(self.histogram < np.inf) else np.inf

    def get_cell_center(self, az_idx: int, el_idx: int) -> tuple[float, float]:
        """返回格子中心的（方位角°，仰角°）"""
        az_deg = (az_idx + 0.5) * self.az_res - 180.0
        el_deg = (el_idx + 0.5) * self.el_res - 90.0
        return az_deg, el_deg

    def cell_to_unit_vector(self, az_idx: int, el_idx: int) -> np.ndarray:
        """
        格子中心方向 → 机体系单位向量 (x, y, z)
        可用于后续速度指令计算。
        """
        az_deg, el_deg = self.get_cell_center(az_idx, el_idx)
        az_rad = np.radians(az_deg)
        el_rad = np.radians(el_deg)
        x =  np.cos(el_rad) * np.cos(az_rad)
        y =  np.cos(el_rad) * np.sin(az_rad)
        z = -np.sin(el_rad)   # Z 正向下，仰角正向上，故取负
        return np.array([x, y, z], dtype=np.float32)

    def select_direction(self, target_vec_body: np.ndarray) -> tuple[np.ndarray, float]:
        """
        从球面直方图中选取最优安全方向（仅水平面方向）。

        限制说明:
            仅在仰角接近 0° 的水平带内搜索候选方向（即 el_idx = el_bins//2 - 1
            与 el_bins//2 这两层，覆盖仰角 [-10°, +10°) 范围），因此返回的
            best_vec 天然位于水平面内（z=0），无需在上层强制置零 vz。

        输入:
            target_vec_body: 目标方向在机体系下的单位向量 (x, y, z)

        返回:
            (best_vec, best_dist)
            best_vec : 最优方向机体系单位向量 (x, y, 0)，严格水平
            best_dist: 最优方向上的最近障碍距离（m），inf 截断为 LIDAR_RANGE_M
        """
        target = np.array(target_vec_body, dtype=np.float64)
        norm = np.linalg.norm(target)
        if norm < 1e-6:
            # 目标方向为零向量（飞机已到达航点），输出零向量
            return np.zeros(3, dtype=np.float32), 0.0

        # 目标方向投影到水平面（去掉 z 分量再归一化），保证水平 VFH 逻辑自洽
        target = target / norm
        target_h = np.array([target[0], target[1], 0.0], dtype=np.float64)
        nh = np.linalg.norm(target_h)
        if nh < 1e-6:
            # 目标几乎正对上/下方，水平面上没有明确朝向：退化为机头方向
            target_h = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            target_h /= nh

        # ── 水平带索引：包含仰角 ≈ 0° 的两层 ──────────────────────
        # el_bins=18 → mid=9 覆盖 [0°,10°)，mid-1=8 覆盖 [-10°,0°)
        el_mid = self.el_bins // 2
        el_lo, el_hi = el_mid - 1, el_mid   # 闭区间

        # ── 预计算水平带内每个方位格子的单位向量（严格水平，z=0）──
        az_centers = np.radians(
            (np.arange(self.az_bins) + 0.5) * self.az_res - 180.0
        )   # shape (36,)
        h_vx = np.cos(az_centers)   # (36,)
        h_vy = np.sin(az_centers)   # (36,)

        # 与目标方向点积（水平）
        dot_h = h_vx * target_h[0] + h_vy * target_h[1]   # (36,)

        # ── 水平带每个方位角的"最短障碍距离"（对两层取 min）───────
        band = self.histogram[:, el_lo:el_hi + 1]   # (36, 2)
        band_dist = band.min(axis=1)                # (36,)

        # ── 区分全堵 / 非全堵 ──────────────────────────────────────
        safe_mask = band_dist > self.exclude_dist   # (36,)
        any_safe  = bool(np.any(safe_mask))

        if any_safe:
            # 非全堵：在安全方位中找点积最大的
            dot_safe = np.where(safe_mask, dot_h, -np.inf)
            best_az = int(np.argmax(dot_safe))
        else:
            # 全堵：找距离最大（最空旷）的方位
            best_az = int(np.argmax(band_dist))

        # ── 邻域加权平均（沿方位角一维，循环边界）─────────────────
        ring = NEIGHBOR_RING
        weight_sum = np.zeros(3, dtype=np.float64)
        total_w    = 0.0

        for daz in range(-ring, ring + 1):
            naz = (best_az + daz) % self.az_bins
            cell_dist = float(band_dist[naz])

            # 非全堵时邻域只使用安全格；全堵时全用
            if any_safe and cell_dist <= self.trigger_dist:
                continue

            # 角度权重：点积截断到 [0,1]
            angle_w = max(0.0, float(dot_h[naz]))

            # 距离权重：截断后归一化
            clamped = min(cell_dist, LIDAR_RANGE_M)
            dist_w  = clamped / LIDAR_RANGE_M

            if any_safe:
                w = angle_w * dist_w
            else:
                # 全堵退化为纯距离权重
                w = dist_w

            vec = np.array([h_vx[naz], h_vy[naz], 0.0], dtype=np.float64)
            weight_sum += w * vec
            total_w    += w

        if total_w < 1e-9:
            # 权重退化：直接用最优方位中心方向
            best_vec = np.array(
                [h_vx[best_az], h_vy[best_az], 0.0], dtype=np.float32,
            )
        else:
            best_vec = (weight_sum / total_w).astype(np.float32)
            best_vec[2] = 0.0   # 严格水平
            n = float(np.linalg.norm(best_vec))
            if n > 1e-9:
                best_vec /= n

        # 最优方位对应的最近障碍距离（inf 截断为量程）
        best_dist = min(float(band_dist[best_az]), LIDAR_RANGE_M)

        return best_vec, best_dist

    # ------------------------------------------------------------------
    # 新接口：只输出 vx 和相对机头的水平偏航偏差（θ_body）
    # ------------------------------------------------------------------
    def compute_vx_yaw(
        self,
        target_vec_body: np.ndarray,
        max_speed: float = MAX_SPEED_MPS,
        lidar_range: float = LIDAR_RANGE_M,
    ) -> tuple[float, float]:
        """
        输入目标方向（机体系单位向量），返回:
            vx           : 机头前向速度大小（m/s，非负）
            yaw_body_off : 机体系下最优水平方向相对机头(+X)的偏差角（rad）
                            = atan2(best_vec_y, best_vec_x)
                            正值为向右（+Y 方向）偏转，负值为向左

        用于策略"vx + 绝对 yaw"：由上层用当前 yaw + yaw_body_off 得到世界系 yaw。
        当已到达目标 / 目标向量为零时，返回 (0.0, 0.0)。
        """
        best_vec, best_dist = self.select_direction(target_vec_body)

        # 若 select_direction 已判定到达（best_vec 为零向量），直接零输出
        n = float(np.linalg.norm(best_vec))
        if n < 1e-6:
            return 0.0, 0.0

        # 速度大小 = min(最优方位障碍距离, lidar_range) / lidar_range * max_speed
        vx = (best_dist / lidar_range) * max_speed

        # 机体系水平偏差角：+X 为机头正前，+Y 为机体右侧
        # atan2(y, x) → 正值表示目标在机体右侧，需要向右偏航
        yaw_body_off = float(math.atan2(best_vec[1], best_vec[0]))

        return float(vx), yaw_body_off

    def print_summary(self):
        """调试用：打印直方图统计信息"""
        finite = self.histogram[self.histogram < np.inf]
        total  = self.histogram.size
        filled = len(finite)
        print(f"[VFH] 格子总数={total}  有点格子={filled}  空格子={total - filled}")
        if filled > 0:
            print(f"      最近障碍={finite.min():.2f}m  "
                  f"最远有效点={finite.max():.2f}m  "
                  f"平均距离={finite.mean():.2f}m")
        print(f"      触发避障: {'是' if self.is_obstacle_near() else '否'} "
              f"（阈值={self.trigger_dist}m）")


# ======================================================================
# 独立运行测试
# ======================================================================
if __name__ == "__main__":
    import sys
    sys.path.insert(0, ".")
    from airsim_lidar import LidarReader
    import time

    lidar = LidarReader()
    vfh   = VFHMap()

    print("[INFO] 开始构建球面直方图，Ctrl+C 退出\n")
    try:
        while True:
            pts = lidar.get_points()
            vfh.build(pts)
            vfh.print_summary()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[STOP] 退出")