# -*- coding: utf-8 -*-
"""
视觉码垛控制系统 —— 分包后的模块化代码

由 test/robot20_sus.py 单文件重构而来，按职责拆分为：
  - common          : 全局串行锁 X5_LOCK、常量、工具函数
  - vision_master   : Vision Master 通信线程（socket 触发/解析）
  - robot_worker    : 机器人命令线程（命令队列 + 串行 SDK 调用 + wait_idle）
  - pallet_logic    : 码垛点生成、趋近/离开位姿等纯计算逻辑
  - vision_calib    : 视觉标定配置/工艺构造、标定 JSON 保存/导入（纯逻辑）
  - ui_main_window  : MainWindow 主界面（界面组装 + 事件槽，调度以上模块）

入口：项目根目录下运行 `python main.py`
"""

__version__ = "1.0.0"
__app_title__ = "视觉码垛控制系统 - 模块化版"

# 对外暴露关键符号，方便外部 `from robot_pallet import X5_LOCK` 等
from .common import X5_LOCK
from .vision_master import VisionMasterThread
from .robot_worker import RobotWorker
from .ui_main_window import MainWindow

__all__ = [
    "X5_LOCK",
    "VisionMasterThread",
    "RobotWorker",
    "MainWindow",
    "__version__",
    "__app_title__",
]
