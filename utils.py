#!/usr/bin/env python3
"""
utils.py — 公共工具函数
"""

import math
import numpy as np


def ned_to_body(vec_ned: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    将 NED 系向量旋转到机体系。
    输入 roll/pitch/yaw 单位为弧度（ZYX 欧拉角）。
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    R = np.array([
        [ cp*cy,          cp*sy,         -sp    ],
        [ sr*sp*cy-cr*sy, sr*sp*sy+cr*cy, sr*cp ],
        [ cr*sp*cy+sr*sy, cr*sp*sy-sr*cy, cr*cp ],
    ], dtype=np.float64)

    return R @ np.array(vec_ned, dtype=np.float64)