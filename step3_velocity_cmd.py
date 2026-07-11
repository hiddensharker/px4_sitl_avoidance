#!/usr/bin/env python3
"""
Arm -> Offboard
1. 起飞到2m（位置控制）
2. 悬停5秒（位置控制）
3. 向北飞5秒（速度控制）
4. 悬停5秒（速度控制）
5. 向东飞5秒（速度控制）
6. 最后悬停（速度控制）
"""

from pymavlink import mavutil
import time

CONNECTION_STRING = "udpout:127.0.0.1:14541"

# -------------------------------
# 参数
# -------------------------------
TAKEOFF_HEIGHT = -2.0      # NED，向上2m

HOVER_TIME = 5.0           # 每次悬停时间(s)

NORTH_SPEED = 1.0          # m/s
EAST_SPEED = 1.0           # m/s

MOVE_TIME = 5.0            # 每段飞行时间(s)

SEND_RATE = 20             # Hz


def connect_px4(conn_str):
    print(f"[INFO] 连接 PX4: {conn_str}")

    master = mavutil.mavlink_connection(conn_str)

    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )

    master.wait_heartbeat()

    print(
        f"[OK] 心跳确认 system={master.target_system}, component={master.target_component}"
    )

    return master


def arm(master):

    print("[INFO] Arm...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )

    while True:

        hb = master.recv_match(
            type="HEARTBEAT",
            blocking=True,
            timeout=3,
        )

        if hb and (
            hb.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        ):
            print("[OK] 已解锁")
            return


#----------------------------------------------------
# 位置控制
#----------------------------------------------------
def send_position(master, x, y, z):

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        # 仅位置有效
        0b0000111111111000,

        x,
        y,
        z,

        0,
        0,
        0,

        0,
        0,
        0,

        0,
        0,
    )


#----------------------------------------------------
# 速度控制
#----------------------------------------------------
def send_velocity(master, vx, vy, vz):

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,

        # 仅速度有效
        0b0000111111000111,

        0,
        0,
        0,

        vx,
        vy,
        vz,

        0,
        0,
        0,

        0,
        0,
    )


def enter_offboard(master, x, y, z):

    print("[INFO] 预发送Offboard Setpoint...")

    for _ in range(40):
        send_position(master, x, y, z)
        time.sleep(1 / SEND_RATE)

    print("[INFO] 切换Offboard...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,
        0,
        0,
        0,
        0,
        0,
    )

    for _ in range(20):
        send_position(master, x, y, z)
        time.sleep(1 / SEND_RATE)

    print("[OK] 已进入Offboard")


def wait_local_position(master):

    while True:
        msg = master.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=True,
            timeout=2,
        )

        if msg is not None:
            return msg


def hold_position(master, x, y, z, duration):

    print(f"[INFO] 悬停 {duration:.1f}s")

    end = time.time() + duration

    while time.time() < end:

        send_position(master, x, y, z)

        time.sleep(1 / SEND_RATE)


def velocity_segment(master, vx, vy, vz, duration, name):

    print(f"[INFO] {name}")

    end = time.time() + duration

    while time.time() < end:

        send_velocity(master, vx, vy, vz)

        time.sleep(1 / SEND_RATE)


def main():

    master = connect_px4(CONNECTION_STRING)

    arm(master)

    print("[INFO] 获取当前位置...")

    msg = wait_local_position(master)

    x = msg.x
    y = msg.y

    enter_offboard(master, x, y, TAKEOFF_HEIGHT)

    # 起飞到2m
    hold_position(master, x, y, TAKEOFF_HEIGHT, 8)

    # 再悬停
    hold_position(master, x, y, TAKEOFF_HEIGHT, HOVER_TIME)

    # 向北飞
    velocity_segment(
        master,
        NORTH_SPEED,
        0.0,
        0.0,
        MOVE_TIME,
        "向北飞"
    )

    # 悬停
    velocity_segment(
        master,
        0.0,
        0.0,
        0.0,
        HOVER_TIME,
        "悬停"
    )

    # 向东飞
    velocity_segment(
        master,
        0.0,
        EAST_SPEED,
        0.0,
        MOVE_TIME,
        "向东飞"
    )

    # 最终悬停
    print("[INFO] 最终悬停")

    while True:

        send_velocity(
            master,
            0.0,
            0.0,
            0.0,
        )

        time.sleep(1 / SEND_RATE)


if __name__ == "__main__":
    main()