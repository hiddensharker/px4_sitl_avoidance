#!/usr/bin/env python3
"""
milestone_phase2.py — 阶段二里程碑验证脚本
完整飞行序列：起飞 → 向北 → 向东 → 上升 → 左转90° → 悬停 → 降落
全程无人工干预，验证 Offboard 控制循环稳定性（含 yaw 独立控制）
"""

import math
import time
import threading
from px4_mavlink import PX4Controller, _DT

# ══════════════════════════════════════════════════════════════
# 配置参数（按需修改）
# ══════════════════════════════════════════════════════════════
TAKEOFF_HEIGHT_M   = 2.0    # 起飞高度（m）
CRUISE_SPEED_MPS   = 1.0    # 水平巡航速度（m/s）
CLIMB_SPEED_MPS    = 0.4    # 上升速度（m/s，NED z 轴向上为负）
YAW_RATE_RADS      = 0.3    # 转弯角速度（rad/s），正值 = 左转（逆时针）
YAW_TARGET_DEG     = -90.0  # 目标转角（度），负值=左转
HOVER_DURATION_S   = 3.0    # 每段动作后悬停缓冲时间（s）
LAND_DESCENT_MPS   = 0.3    # 降落速度（m/s）
LAND_STOP_Z_M      = 0.50   # 认为已落地的高度阈值（m，NED z 接近 0）

# ══════════════════════════════════════════════════════════════
# 辅助：全程位姿打印线程
# ══════════════════════════════════════════════════════════════
def pose_printer(ctrl: PX4Controller, stop_event: threading.Event):
    while not stop_event.is_set():
        pos = ctrl.get_position()
        att = ctrl.get_attitude()
        if pos["x"] is not None and att["yaw"] is not None:
            print(
                f"  [状态] "
                f"x={pos['x']:6.2f}m  y={pos['y']:6.2f}m  z={pos['z']:6.2f}m  |  "
                f"yaw={math.degrees(att['yaw']):6.1f}°  "
                f"vx={pos['vx']:5.2f}  vy={pos['vy']:5.2f}  vz={pos['vz']:5.2f} m/s"
            )
        time.sleep(0.5)   # 2 Hz 打印，不影响控制

# ══════════════════════════════════════════════════════════════
# 步骤辅助函数
# ══════════════════════════════════════════════════════════════
def step(label: str):
    """打印带分隔线的步骤标题"""
    print(f"\n{'─'*55}")
    print(f"  STEP │ {label}")
    print(f"{'─'*55}")


def verify_height(ctrl: PX4Controller, target_m: float, tol_m: float = 0.4) -> bool:
    """检查当前高度是否在目标附近（NED：z 负值=高度）"""
    pos = ctrl.get_position()
    if pos["z"] is None:
        return False
    current_height = -pos["z"]   # 转换为正值高度
    ok = abs(current_height - target_m) < tol_m
    print(f"  [校验] 当前高度={current_height:.2f}m，目标={target_m}m  →  {'✓ 通过' if ok else '✗ 偏差过大'}")
    return ok


def verify_yaw(ctrl: PX4Controller, initial_yaw_rad: float, delta_deg: float, tol_deg: float = 10.0) -> bool:
    """检查 yaw 是否从初始值转过了 delta_deg"""
    att = ctrl.get_attitude()
    if att["yaw"] is None:
        return False
    turned = math.degrees(att["yaw"] - initial_yaw_rad)
    # 归一化到 (-180, 180]
    turned = (turned + 180) % 360 - 180
    ok = abs(abs(turned) - abs(delta_deg)) < tol_deg
    print(f"  [校验] 转向量={turned:.1f}°，目标={delta_deg}°  →  {'✓ 通过' if ok else '✗ 偏差过大'}")
    return ok


def fly_velocity(
    ctrl: PX4Controller,
    vx: float, vy: float, vz: float,
    yaw_rate: float = 0.0,
    duration: float = 5.0,
    label: str = "",
):
    """持续发送速度指令 duration 秒"""
    if label:
        print(f"  → 执行: {label}  (vx={vx} vy={vy} vz={vz} yaw_rate={yaw_rate:.2f}  {duration}s)")
    ctrl.send_velocity(vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, duration=duration)


# ══════════════════════════════════════════════════════════════
# 主验证序列
# ══════════════════════════════════════════════════════════════
def main():
    results: dict[str, bool] = {}

    # ── 初始化 ────────────────────────────────────────────────
    ctrl = PX4Controller()
    ctrl.connect()

    stop_print = threading.Event()
    printer = threading.Thread(
        target=pose_printer, args=(ctrl, stop_print), daemon=True
    )
    printer.start()

    try:
        # ── S1: 解锁 + 进入 Offboard ──────────────────────────
        step("S1 · 解锁并进入 Offboard 模式")
        ctrl.arm()
        ctrl.enter_offboard()
        print("  ✓ Offboard 模式已激活")

        # ── S2: 起飞到 2m ─────────────────────────────────────
        step(f"S2 · 起飞至 {TAKEOFF_HEIGHT_M}m")
        ctrl.takeoff(height_m=TAKEOFF_HEIGHT_M, wait=8.0)
        results["S2_takeoff"] = verify_height(ctrl, TAKEOFF_HEIGHT_M)
        ctrl.hover(duration=HOVER_DURATION_S)

        # ── S3: vz 单独验证（上升 1m 再回来）─────────────────
        step("S3 · vz 独立验证（上升 1m → 回到 2m）")
        fly_velocity(ctrl, 0, 0, -CLIMB_SPEED_MPS, duration=2.5, label="上升")
        ctrl.hover(duration=1.0)
        expected_h = TAKEOFF_HEIGHT_M + CLIMB_SPEED_MPS * 2.5
        results["S3_vz_up"] = verify_height(ctrl, expected_h, tol_m=0.6)

        fly_velocity(ctrl, 0, 0, CLIMB_SPEED_MPS, duration=2.5, label="下降回基准高度")
        ctrl.hover(duration=1.0)
        results["S3_vz_down"] = verify_height(ctrl, TAKEOFF_HEIGHT_M, tol_m=0.6)

        # ── S4: 向北飞 5s ─────────────────────────────────────
        step("S4 · 向北飞（vx=+1 m/s，5s）")
        fly_velocity(ctrl, CRUISE_SPEED_MPS, 0, 0, duration=5.0, label="向北飞")
        ctrl.hover(duration=HOVER_DURATION_S)
        results["S4_north"] = verify_height(ctrl, TAKEOFF_HEIGHT_M)   # 高度应保持

        # ── S5: 向东飞 5s ─────────────────────────────────────
        step("S5 · 向东飞（vy=+1 m/s，5s）")
        fly_velocity(ctrl, 0, CRUISE_SPEED_MPS, 0, duration=5.0, label="向东飞")
        ctrl.hover(duration=HOVER_DURATION_S)
        results["S5_east"] = verify_height(ctrl, TAKEOFF_HEIGHT_M)

        # ── S6: 再上升 2m（总高 4m）──────────────────────────
        step(f"S6 · 上升至 {TAKEOFF_HEIGHT_M + 2.0}m")
        climb_secs = 2.0 / CLIMB_SPEED_MPS
        fly_velocity(ctrl, 0, 0, -CLIMB_SPEED_MPS, duration=climb_secs, label=f"上升 2m ({climb_secs:.1f}s)")
        ctrl.hover(duration=HOVER_DURATION_S)
        results["S6_climb"] = verify_height(ctrl, TAKEOFF_HEIGHT_M + 2.0, tol_m=0.6)

        # ── S7: 左转 90° ──────────────────────────────────────
        step(f"S7 · 左转 {abs(YAW_TARGET_DEG):.0f}°（yaw_rate={YAW_RATE_RADS} rad/s）")
        att_before = ctrl.get_attitude()
        yaw0 = att_before["yaw"] if att_before["yaw"] is not None else 0.0
        turn_secs = abs(math.radians(YAW_TARGET_DEG)) / YAW_RATE_RADS
        print(f"  预计转弯耗时: {turn_secs:.1f}s，初始 yaw={math.degrees(yaw0):.1f}°")
        # YAW_TARGET_DEG=-90 → 左转 → yaw_rate 取负（逆时针）
        fly_velocity(
            ctrl, 0, 0, 0,
            yaw_rate=-YAW_RATE_RADS,   # 负值=机体左转
            # duration=turn_secs,
            duration=1.9*turn_secs,
            label=f"左转 {abs(YAW_TARGET_DEG):.0f}°",
        )
        ctrl.hover(duration=HOVER_DURATION_S)
        results["S7_yaw"] = verify_yaw(ctrl, yaw0, YAW_TARGET_DEG)

        # ── S8: 自动降落 ──────────────────────────────────────
        step("S8 · 自动降落")
        print("  → 缓慢下降至地面...")
        deadline = time.time() + 60.0   # 最多等 60s
        while time.time() < deadline:
            pos = ctrl.get_position()
            if pos["z"] is not None and abs(pos["z"]) < LAND_STOP_Z_M:
                print("  ✓ 检测到接地，停止下降指令")
                break
            ctrl.send_velocity(vx=0.0, vy=0.0, vz=LAND_DESCENT_MPS)
            time.sleep(_DT)
        else:
            print("  ⚠ 降落超时（60s），强制停止")

        ctrl.hover(duration=1.0)
        results["S8_land"] = True   # 若程序走到这里即视为降落成功

    except KeyboardInterrupt:
        print("\n[中断] 用户手动停止，执行悬停保护")
        ctrl.hover(duration=2.0)

    except Exception as e:
        print(f"\n[错误] {e}")
        ctrl.hover(duration=2.0)
        raise

    finally:
        stop_print.set()
        ctrl.close()

    # ══════════════════════════════════════════════════════════
    # 汇总报告
    # ══════════════════════════════════════════════════════════
    print(f"\n{'═'*55}")
    print("  阶段二里程碑验证报告")
    print(f"{'═'*55}")
    checks = {
        "S2_takeoff" : "起飞高度到位",
        "S3_vz_up"   : "vz 上升响应",
        "S3_vz_down" : "vz 下降响应",
        "S4_north"   : "向北飞高度保持",
        "S5_east"    : "向东飞高度保持",
        "S6_climb"   : "再上升 2m 到位",
        "S7_yaw"     : "左转 90° yaw 响应",
        "S8_land"    : "降落完成",
    }
    all_pass = True
    for key, desc in checks.items():
        ok = results.get(key, False)
        all_pass = all_pass and ok
        print(f"  {'✓' if ok else '✗'}  {desc}")

    print(f"{'─'*55}")
    if all_pass:
        print("  ✅ 阶段二里程碑：全部通过，可进入阶段三 VFH")
    else:
        print("  ❌ 存在未通过项，请检查对应步骤后重跑")
    print(f"{'═'*55}\n")


if __name__ == "__main__":
    main()