# -*- coding: utf-8 -*-
"""
视觉码垛控制系统 —— 模块化版入口。

对 test/robot20_sus.py 成功演示版本进行重构，模块代码存在 robot_pallet/ 文件夹里面：
  - robot_pallet/vision_master.py : Vision Master 通信线程（海康软件）
  - robot_pallet/robot_worker.py  : 机器人命令线程  
  - robot_pallet/pallet_logic.py  : 码垛点生成等纯计算逻辑
  - robot_pallet/vision_calib.py  : 视觉标定配置/工艺构造、标定 JSON 读写
  - robot_pallet/ui_main_window.py: MainWindow 主界面
  - robot_pallet/common.py        : 全局串行锁 X5_LOCK、常量、工具函数

主入口：
    python main.py
"""
import os
import sys

# 保证无论从哪个目录运行，都能导入项目根目录下的 robot_pallet 包
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import QApplication

from robot_pallet import MainWindow


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
