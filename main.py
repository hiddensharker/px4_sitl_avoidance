#!/usr/bin/env python3
"""
main.py — VFH 避障主控循环
阶段三子任务四：Mission ↔ Offboard 干预模式，静态障碍绕开测试
"""

import math
import time
import threading
import numpy as np

from airsim_lidar import LidarReader
from px4_mavlink   import PX4Controller, _DT
from vfh           import VFHMap
from utils    import ned_to_body
from config import CONTROL_HZ, CONTROL_DT, TRIGGER_DIST_M, MAX_SPEED_MPS, LIDAR_RANGE_M

# ======================================================================
# 状态打印线程
# ======================================================================
def status_printer(
    ctrl: PX4Controller,
    stop_event: threading.Event,
    mode_ref: list,
    saved_target_ref: list,   # saved_target_ref[0] = 当前保存的目标航点
):
    while not stop_event.is_set():
        pos = ctrl.get_position()
        att = ctrl.get_attitude()
        wp  = saved_target_ref[0]
        if pos["x"] is not None and att["yaw"] is not None:
            print(
                f"  [状态/{mode_ref[0]:8s}] "
                f"pos=({pos['x']:6.2f},{pos['y']:6.2f},{pos['z']:6.2f})m  "
                f"yaw={math.degrees(att['yaw']):6.1f}°  "
                f"wp=({wp['x']},{wp['y']},{wp['z']})"
            )
        time.sleep(0.5)


# ======================================================================
# 主控循环
# ======================================================================
def main():
    lidar = LidarReader()
    ctrl  = PX4Controller()
    vfh   = VFHMap(trigger_dist_m=TRIGGER_DIST_M)

    ctrl.connect()
    ctrl.fetch_mission()   # 拉取 QGC 任务航点

    mode_ref         = ["MISSION"]
    in_offboard      = False
    saved_target     = {"x": None, "y": None, "z": None}
    saved_target_ref = [saved_target]   # 供打印线程读取

    stop_print = threading.Event()
    printer = threading.Thread(
        target=status_printer,
        args=(ctrl, stop_print, mode_ref, saved_target_ref),
        daemon=True,
    )
    printer.start()

    print("[MAIN] 等待 PX4 位姿数据...")
    ctrl.wait_position()
    print("[MAIN] 位姿数据就绪，开始主控循环（Mission 模式巡航）\n")

    try:
        while True:
            t0 = time.time()

            # ── 1. 读取点云，构建球面直方图 ───────────────────
            pts = lidar.get_points()
            vfh.build(pts)

            obstacle_near = vfh.is_obstacle_near()

            # ── 2. 模式判断与切换 ─────────────────────────────
            current_wp = ctrl.get_current_waypoint_ned()

            if obstacle_near and not in_offboard and current_wp["x"] is not None:
                # 触发避障：保存当前真实航点，切入 Offboard
                saved_target = current_wp
                saved_target_ref[0] = saved_target
                print(f"[MAIN] ⚠ 检测到障碍物，切入 Offboard 避障模式")
                print(f"[MAIN] 保存真实航点 seq={ctrl._mission_current} "
                      f"NED=({saved_target['x']:.1f},{saved_target['y']:.1f},{saved_target['z']:.1f})")
                ctrl.enter_offboard()
                in_offboard = True
                mode_ref[0] = "OFFBOARD"

            elif not obstacle_near and in_offboard:
                # 解除触发：切回 Mission
                print("[MAIN] ✓ 障碍物已清除，切回 Mission 模式")
                ctrl.exit_offboard()
                in_offboard = False
                mode_ref[0] = "MISSION"

            # ── 3. Offboard 模式下输出 VFH 速度指令 ──────────
            if in_offboard:
                pos = ctrl.get_position()
                att = ctrl.get_attitude()

                # 检查数据有效性
                if (pos["x"] is None or att["yaw"] is None
                        or saved_target["x"] is None):
                    ctrl.send_velocity(0.0, 0.0, 0.0)
                else:
                    # 3a. 计算目标方向向量（世界 NED 系）
                    target_ned = np.array([
                        saved_target["x"] - pos["x"],
                        saved_target["y"] - pos["y"],
                        saved_target["z"] - pos["z"],
                    ], dtype=np.float64)

                    dist_to_target = float(np.linalg.norm(target_ned))

                    if dist_to_target < 0.5:
                        ctrl.send_velocity(0.0, 0.0, 0.0)
                    else:
                        # 3b. 目标方向单位向量：NED 系 → 机体系
                        target_ned_unit = target_ned / dist_to_target
                        target_body = ned_to_body(
                            target_ned_unit,
                            att["roll"],
                            att["pitch"],
                            att["yaw"],
                        ).astype(np.float32)

                        # 3c. VFH 计算：仅得到 vx（机头前向速度）与
                        #     机体系水平偏差角 θ_body（+右/-左）
                        vx, yaw_body_off = vfh.compute_vx_yaw(
                            target_body,
                            max_speed=MAX_SPEED_MPS,
                            lidar_range=LIDAR_RANGE_M,
                        )

                        # 3d. 组装世界系绝对 yaw = 当前 yaw + θ_body，
                        #     归一化到 [-π, π]。yaw 语义为 NED 下相对正北的
                        #     绝对航向角（顺时针为正）。
                        yaw_cmd = att["yaw"] + yaw_body_off
                        yaw_cmd = math.atan2(math.sin(yaw_cmd), math.cos(yaw_cmd))

                        print(
                            f"  [VFH] vx={vx:5.2f} m/s  "
                            f"θ_body={math.degrees(yaw_body_off):+6.1f}°  "
                            f"yaw_cmd={math.degrees(yaw_cmd):+6.1f}°  "
                            f"目标距离={dist_to_target:.1f}m"
                        )

                        # 3e. 仅下发前向速度 + 绝对 yaw，vy=vz=0
                        ctrl.send_velocity(vx=vx, vy=0.0, vz=0.0, yaw=yaw_cmd)

            # ── 4. 控制频率维持 ───────────────────────────────
            elapsed = time.time() - t0
            sleep_t = CONTROL_DT - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[MAIN] 用户中断，悬停保护")
        if in_offboard:
            ctrl.hover(duration=2.0)

    except Exception as e:
        print(f"\n[MAIN] 异常: {e}")
        if in_offboard:
            ctrl.hover(duration=2.0)
        raise

    finally:
        stop_print.set()
        ctrl.close()


if __name__ == "__main__":
    main()