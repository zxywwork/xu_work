# -*- coding: utf-8 -*-
"""
全局公共模块：串行锁、常量、路径与点位构造工具。

核心约定（来自原 robot20_sus.py 多线程修复经验）：
  - x5/x5v 原生 SDK 非线程安全，所有线程（主线程、RobotWorker 线程、
    连续运行线程、视觉码垛线程）的 SDK 调用都必须经过 X5_LOCK，
    避免并发调用导致闪退。
  - 不要在持有 X5_LOCK 期间 time.sleep 或弹 QMessageBox（会卡住其他线程），
    应逐个调用加锁。
"""
import os
from threading import Lock

import xapi.api as x5

# ---------------------------------------------------------------------------
# 全局串行锁：x5 / x5v 原生 SDK 非线程安全，所有线程的 SDK 调用必须经过它
# ---------------------------------------------------------------------------
X5_LOCK = Lock()

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------
# 系统模式名称映射
MODE_NAMES = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}

# 视觉工艺号数量（界面 VISION 0~7）
VISION_COUNT = 8

# 码垛参考点在机器人侧的 PR 编号
PR_P1 = 10
PR_P2 = 11
PR_P3 = 12
PR_PHOTO = 14
PR_SAFE = 20

# 项目根目录（robot_pallet 包的上一级）与常用路径
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(PROJECT_ROOT, "logs")
CALIB_FILE_PATH = os.path.join(PROJECT_ROOT, "vision_calib.json")

# ---------------------------------------------------------------------------
# 点位 / 位姿构造工具
# ---------------------------------------------------------------------------
def build_pose(x, y, z, a=0.0, b=0.0, c=0.0):
    """构造 x5.Pose（后 3 个虚拟轴参数固定为 0）。"""
    return x5.Pose(x, y, z, a, b, c, 0, 0, 0)


def build_point(pose, uf=0, tf=0, cfg=(0, 0, 0, 1)):
    """构造 x5.Point。"""
    return x5.Point(pose=pose, uf=uf, tf=tf, cfg=cfg)
