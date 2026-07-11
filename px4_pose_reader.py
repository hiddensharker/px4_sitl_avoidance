#!/usr/bin/env python3
"""
连接 PX4 SITL，订阅并打印位姿数据 (位置 + 姿态)
依赖: pip install pymavlink
"""

from pymavlink import mavutil
import time

# PX4 SITL 默认 MAVLink 转发端口（用于外部脚本连接，避免占用 QGC 的 14550）
# 若用的是 PX4 自带 SITL（非 jMAVSim/Gazebo 特殊配置），常见端口：
#   udp:14540  -> offboard/companion 常用端口
#   udp:14550  -> QGC 常用端口（多个端口可同时连接，互不影响）
CONNECTION_STRING = "udpout:127.0.0.1:14541"


def connect_px4(conn_str: str):
    print(f"[INFO] 正在连接 PX4: {conn_str}")
    master = mavutil.mavlink_connection(conn_str)

    # udpout 模式下 pymavlink 不会自动发包，必须主动发送一个心跳"破冰"，
    # 否则双方都在等对方先发包，永远卡住
    print("[INFO] 主动发送心跳破冰...")
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    master.wait_heartbeat()
    print(f"[INFO] 心跳确认: system={master.target_system}, component={master.target_component}")
    return master


def request_data_streams(master):
    """部分 PX4 固件需要主动请求消息频率，新版本默认会推送，这里做兼容处理"""
    # 请求 LOCAL_POSITION_NED 和 ATTITUDE 以一定频率推送（单位: Hz）
    msg_ids = {
        "LOCAL_POSITION_NED": 32,
        "ATTITUDE": 30,
    }
    for name, msg_id in msg_ids.items():
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            100000,  # 100ms = 10Hz，单位微秒
            0, 0, 0, 0, 0
        )
    print("[INFO] 已请求 LOCAL_POSITION_NED / ATTITUDE 数据流")


def main():
    master = connect_px4(CONNECTION_STRING)
    request_data_streams(master)

    pos = {"x": None, "y": None, "z": None}
    att = {"roll": None, "pitch": None, "yaw": None}

    print("[INFO] 开始接收位姿数据 (Ctrl+C 退出)...")
    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == "LOCAL_POSITION_NED":
                pos["x"] = msg.x
                pos["y"] = msg.y
                pos["z"] = msg.z
                print(f"[POS] x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f} "
                      f"vx={msg.vx:.3f} vy={msg.vy:.3f} vz={msg.vz:.3f}")

            elif msg_type == "ATTITUDE":
                att["roll"] = msg.roll
                att["pitch"] = msg.pitch
                att["yaw"] = msg.yaw
                print(f"[ATT] roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}")

    except KeyboardInterrupt:
        print("\n[INFO] 退出")


if __name__ == "__main__":
    main()