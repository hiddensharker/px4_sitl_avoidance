#!/usr/bin/env python3
"""
px4_mavlink.py — PX4 MAVLink 连接 / 位姿订阅 / Offboard 控制封装
依赖: pip install pymavlink
"""

import math
import time
import threading
from pymavlink import mavutil


# ======================================================================
# 常量
# ======================================================================
DEFAULT_CONN = "udpout:127.0.0.1:14541"
SEND_RATE_HZ = 20
_DT = 1.0 / SEND_RATE_HZ


# ======================================================================
# PX4Controller
# ======================================================================
class PX4Controller:

    def __init__(self, connection_string: str = DEFAULT_CONN):
        self.conn_str = connection_string
        self.master = None

        self._pos = {"x": None, "y": None, "z": None,
                     "vx": None, "vy": None, "vz": None}
        self._att = {"roll": None, "pitch": None, "yaw": None}
        self._pos_target = {"x": None, "y": None, "z": None}

        # 任务航点缓存
        self._mission_items = {}   # seq -> {"lat", "lon", "alt"}
        self._mission_current = 0
        self._home_lat = None
        self._home_lon = None
        self._home_alt = None

        self._pose_lock = threading.Lock()

        self._offboard_active = False
        self._watchdog_thread = None
        self._watchdog_setpoint = (0.0, 0.0, 0.0)
        self._watchdog_mode = "velocity"
        self._watchdog_yaw = None      # None = 忽略 yaw；数值 = 世界系绝对 yaw
        self._watchdog_yaw_rate = 0.0

    # ------------------------------------------------------------------
    # 连接
    # ------------------------------------------------------------------
    def connect(self):
        print(f"[PX4] 连接 {self.conn_str} ...")
        self.master = mavutil.mavlink_connection(self.conn_str)

        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        self.master.wait_heartbeat()
        print(f"[PX4] 连接成功 system={self.master.target_system} "
              f"component={self.master.target_component}")

        self._request_streams()
        # 注意：pose 监听线程放到 fetch_mission() 结束后再启动，
        # 避免它抢先把 MISSION_COUNT / MISSION_ITEM_INT / HOME_POSITION
        # 等一次性任务消息吃掉，导致 fetch_mission 里 recv_match 超时。

    def close(self):
        self._offboard_active = False
        if self.master:
            self.master.close()
        print("[PX4] 连接已关闭")

    # ------------------------------------------------------------------
    # 任务航点获取
    # ------------------------------------------------------------------
    def fetch_mission(self):
        """
        拉取 QGC 任务列表并缓存，同时获取 HOME 点用于坐标转换。
        必须在 connect() 之后、主循环之前调用。
        """
        # 1. 获取 HOME 点
        print("[PX4] 请求 HOME 点...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

        home = self.master.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
        if home is None:
            raise TimeoutError("[PX4] 未收到 HOME_POSITION，请确认飞机已上锁解锁过")
        self._home_lat = home.latitude  * 1e-7
        self._home_lon = home.longitude * 1e-7
        self._home_alt = home.altitude  * 1e-3  # mm → m
        print(f"[PX4] HOME: lat={self._home_lat:.6f} lon={self._home_lon:.6f} "
              f"alt={self._home_alt:.1f}m")

        # 2. 请求任务数量
        print("[PX4] 请求任务列表...")
        self.master.mav.mission_request_list_send(
            self.master.target_system,
            self.master.target_component,
        )
        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if msg is None:
            raise TimeoutError("[PX4] 未收到 MISSION_COUNT")
        count = msg.count
        print(f"[PX4] 任务共 {count} 个航点")

        # 3. 逐个请求航点
        self._mission_items.clear()
        for seq in range(count):
            self.master.mav.mission_request_int_send(
                self.master.target_system,
                self.master.target_component,
                seq,
            )
            item = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if item is None:
                print(f"[PX4] 航点 {seq} 请求超时，跳过")
                continue
            lat = item.x * 1e-7
            lon = item.y * 1e-7
            alt = item.z          # 相对高度，米
            self._mission_items[seq] = {"lat": lat, "lon": lon, "alt": alt}
            print(f"[PX4] 航点 {seq}: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}m")

        print(f"[PX4] 任务加载完成，共 {len(self._mission_items)} 个有效航点")

        # 新增：主动请求当前航点序号
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            42,
            0, 0, 0, 0, 0, 0,
        )
        msg = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
        if msg:
            self._mission_current = msg.seq
            print(f"[PX4] 当前航点序号: {self._mission_current}")
        else:
            print("[PX4] 未收到 MISSION_CURRENT，序号保持为 0")

        # 所有一次性任务消息拉取完成后，再启动持续 pose 监听线程
        self._start_pose_listener()

    def _latlon_to_ned(self, lat, lon, alt):
        """全局坐标 → 本地 NED 坐标（相对 HOME 点）"""
        R = 6371000.0
        dlat = math.radians(lat - self._home_lat)
        dlon = math.radians(lon - self._home_lon)
        x = dlat * R
        y = dlon * R * math.cos(math.radians(self._home_lat))
        z = -(alt - self._home_alt)
        return x, y, z

    def get_current_waypoint_ned(self) -> dict:
        """
        返回当前任务航点的 NED 坐标 {'x', 'y', 'z'}。
        若任务未加载或航点序号无效则各字段为 None。
        """
        with self._pose_lock:
            seq = self._mission_current
        item = self._mission_items.get(seq)
        if item is None:
            return {"x": None, "y": None, "z": None}
        x, y, z = self._latlon_to_ned(item["lat"], item["lon"], item["alt"])
        return {"x": x, "y": y, "z": z}

    # ------------------------------------------------------------------
    # 位姿读取
    # ------------------------------------------------------------------
    def get_position(self) -> dict:
        with self._pose_lock:
            return dict(self._pos)

    def get_attitude(self) -> dict:
        with self._pose_lock:
            return dict(self._att)

    def get_position_target(self) -> dict:
        with self._pose_lock:
            return dict(self._pos_target)

    def wait_position(self, timeout: float = 5.0) -> dict:
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.get_position()
            if p["x"] is not None:
                return p
            time.sleep(0.05)
        raise TimeoutError("[PX4] 等待位置数据超时")

    # ------------------------------------------------------------------
    # Arm / Disarm
    # ------------------------------------------------------------------
    def arm(self, timeout: float = 10.0):
        print("[PX4] Arm...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[PX4] 已解锁 (Armed)")
                return
        raise TimeoutError("[PX4] Arm 超时")

    def disarm(self):
        print("[PX4] Disarm...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    # ------------------------------------------------------------------
    # Offboard 模式
    # ------------------------------------------------------------------
    def enter_offboard(self, pre_send_count: int = 40):
        pos = self.wait_position()
        x, y = pos["x"], pos["y"]
        z = pos["z"]                          # ← 新增：取当前高度

        print("[PX4] 预发送 Offboard setpoint...")
        for _ in range(pre_send_count):
            self._send_position_ned(x, y, -2.0)
            time.sleep(_DT)

        print("[PX4] 切换 Offboard 模式...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,
            0, 0, 0, 0, 0,
        )
        for _ in range(20):
            self._send_position_ned(x, y, z)
            time.sleep(_DT)

        self._offboard_active = True
        self._start_watchdog(x, y, z, mode="position")
        print("[PX4] 已进入 Offboard 模式")

    def exit_offboard(self):
        print("[PX4] 退出 Offboard，切回 Mission 模式...")
        self._offboard_active = False
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,
            0, 0, 0, 0, 0,
        )
        print("[PX4] 已发送切回 Mission 指令")

    # ------------------------------------------------------------------
    # 高层指令接口
    # ------------------------------------------------------------------
    def takeoff(self, height_m: float = 2.0, wait: float = 8.0):
        pos = self.get_position()
        x, y = pos["x"], pos["y"]
        z = -abs(height_m)
        print(f"[PX4] 起飞至 {height_m}m ...")
        self._update_watchdog(x, y, z, mode="position")
        self._hold_position(x, y, z, wait)

    def hover(self, duration: float = 5.0):
        print(f"[PX4] 悬停 {duration:.1f}s")
        self._update_watchdog(0.0, 0.0, 0.0, mode="velocity")
        end = time.time() + duration
        while time.time() < end:
            self._send_velocity_ned(0.0, 0.0, 0.0, 0.0)
            time.sleep(_DT)

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float = 0.0,
        yaw: float = None,
        yaw_rate: float = 0.0,
        duration: float = 0.0,
        label: str = "",
    ):
        """
        下发机体系 (MAV_FRAME_BODY_NED) 速度指令。

        yaw / yaw_rate 参数为互斥关系:
            yaw  = 数值(rad)  -> 世界系(NED)下相对正北的绝对偏航角，PX4 会自旋到该航向
            yaw  = None       -> 仅使用 yaw_rate（原行为）
        """
        if label:
            print(f"[PX4] 速度指令: {label}  vx={vx} vy={vy} vz={vz} "
                  f"yaw={yaw}  yaw_rate={yaw_rate}")
        self._update_watchdog(vx, vy, vz, mode="velocity",
                              yaw=yaw, yaw_rate=yaw_rate)

        if duration <= 0:
            self._send_velocity_ned(vx, vy, vz, yaw_rate=yaw_rate, yaw=yaw)
            return

        end = time.time() + duration
        while time.time() < end:
            self._send_velocity_ned(vx, vy, vz, yaw_rate=yaw_rate, yaw=yaw)
            time.sleep(_DT)

    def goto_position(self, x: float, y: float, z_ned: float, duration: float = 8.0):
        print(f"[PX4] 飞往位置 x={x:.2f} y={y:.2f} z={z_ned:.2f}")
        self._update_watchdog(x, y, z_ned, mode="position")
        self._hold_position(x, y, z_ned, duration)

    # ------------------------------------------------------------------
    # 私有: MAVLink 原始发送
    # ------------------------------------------------------------------
    def _send_position_ned(self, x, y, z):
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0,
        )

    def _send_velocity_ned(self, vx, vy, vz, yaw_rate=0.0, yaw=None):
        """
        底层封装：向 PX4 发送 SET_POSITION_TARGET_LOCAL_NED（BODY_NED frame）。

        type_mask bit 分配 (置 1 表示忽略):
            bit 0-2 : x, y, z (position)
            bit 3-5 : vx, vy, vz
            bit 6-8 : ax, ay, az
            bit 9   : force
            bit 10  : yaw
            bit 11  : yaw_rate

        - yaw=None      : 使用 yaw_rate（忽略 yaw），mask = 0b0000_1100_1100_0111
        - yaw=数值(rad) : 使用绝对 yaw（忽略 yaw_rate），mask = 0b0000_1000_1100_0111
                          此时 yaw 是世界系 NED 下相对正北的绝对航向角
        """
        if yaw is None:
            # 忽略 position(0-2) / accel(6-8) / force(9) / yaw(10)，使用 vx,vy,vz + yaw_rate
            type_mask = 0b0000_1100_1100_0111
            yaw_field = 0.0
            yr_field  = yaw_rate
        else:
            # 忽略 position(0-2) / accel(6-8) / force(9) / yaw_rate(11)，使用 vx,vy,vz + yaw
            type_mask = 0b0000_1000_1100_0111
            yaw_field = float(yaw)
            yr_field  = 0.0

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            yaw_field, yr_field,
        )

    # ------------------------------------------------------------------
    # 私有: 辅助
    # ------------------------------------------------------------------
    def _hold_position(self, x, y, z, duration):
        end = time.time() + duration
        while time.time() < end:
            self._send_position_ned(x, y, z)
            time.sleep(_DT)

    def _request_streams(self):
        for msg_id in (32, 30, 85):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                100_000,
                0, 0, 0, 0, 0,
            )
        # MISSION_CURRENT 1 Hz 足够
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            42,
            1_000_000,
            0, 0, 0, 0, 0,
        )
        print("[PX4] 已请求 LOCAL_POSITION_NED / ATTITUDE / POSITION_TARGET_LOCAL_NED 数据流")

    def _start_pose_listener(self):
        def _loop():
            while True:
                msg = self.master.recv_match(blocking=True, timeout=1)
                if msg is None:
                    continue
                t = msg.get_type()
                if t == "LOCAL_POSITION_NED":
                    with self._pose_lock:
                        self._pos.update(
                            x=msg.x, y=msg.y, z=msg.z,
                            vx=msg.vx, vy=msg.vy, vz=msg.vz,
                        )
                elif t == "ATTITUDE":
                    with self._pose_lock:
                        self._att.update(
                            roll=msg.roll,
                            pitch=msg.pitch,
                            yaw=msg.yaw,
                        )
                elif t == "POSITION_TARGET_LOCAL_NED":
                    if not (math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.z)):
                        with self._pose_lock:
                            self._pos_target.update(
                                x=msg.x, y=msg.y, z=msg.z,
                            )
                elif t == "MISSION_CURRENT":
                    with self._pose_lock:
                        self._mission_current = msg.seq

        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    def _update_watchdog(self, a, b, c, mode, yaw=None, yaw_rate=0.0):
        self._watchdog_setpoint = (a, b, c)
        self._watchdog_mode = mode
        self._watchdog_yaw = yaw
        self._watchdog_yaw_rate = yaw_rate

    def _start_watchdog(self, a, b, c, mode):
        self._update_watchdog(a, b, c, mode)

        def _loop():
            while self._offboard_active:
                sp = self._watchdog_setpoint
                m  = self._watchdog_mode
                if m == "velocity":
                    self._send_velocity_ned(
                        *sp,
                        yaw_rate=getattr(self, "_watchdog_yaw_rate", 0.0),
                        yaw=getattr(self, "_watchdog_yaw", None),
                    )
                else:
                    self._send_position_ned(*sp)
                time.sleep(_DT)

        self._watchdog_thread = threading.Thread(target=_loop, daemon=True)
        self._watchdog_thread.start()
        print("[PX4] Offboard watchdog 已启动")
