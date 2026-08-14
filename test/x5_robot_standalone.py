import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer

# 导入 xapi 机器人库
import xapi.api as x5


# ---------- 工作线程：专门负责与 X5 机器人通信 ----------
class RobotThread(QThread):
    # 信号
    connected_signal = pyqtSignal(bool)        # 连接成功/失败
    state_updated = pyqtSignal(dict)           # 状态数据更新
    log_signal = pyqtSignal(str)               # 日志消息

    def __init__(self):
        super().__init__()
        self.handle = None
        self.ip = ""
        self.running = False
        self.update_interval = 200  # 毫秒

    def set_ip(self, ip):
        self.ip = ip

    def connect_robot(self):
        """连接机器人（由主线程调用）"""
        try:
            self.handle = x5.connect(self.ip)
            self.running = True
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[机器人] 连接成功，句柄: {self.handle}")
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[机器人] 连接失败: {str(e)}")
            self.handle = None

    def disconnect_robot(self):
        """断开机器人"""
        self.running = False
        if self.handle is not None:
            try:
                x5.disconnect(self.handle)
            except:
                pass
            self.handle = None
        self.connected_signal.emit(False)
        self.log_signal.emit("[机器人] 已断开连接")

    def get_state(self):
        """获取一次状态并发射信号"""
        if self.handle is None:
            return
        try:
            state = x5.get_system_state(self.handle)
            speed = x5.get_speed(self.handle)
            joint = x5.get_cjoint(self.handle)
            point = x5.get_cpoint(self.handle)

            info = {
                'mode': state.mode,          # 2自动,3手动,5调试,100自动命令
                'enable': state.enable,      # 0/1
                'alarm': state.alarm,        # 0正常
                'remote': state.remote,      # 0本地,1远程
                'run': state.run,            # 1运行,2暂停,3停止
                'in_pos': state.in_pos,
                'speed': speed,
                'joint': (joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6),
                'cart': (point.pose.x, point.pose.y, point.pose.z, 
                         point.pose.a, point.pose.b, point.pose.c)
            }
            self.state_updated.emit(info)
        except Exception as e:
            self.log_signal.emit(f"[状态获取异常] {str(e)}")

    # ---------- 控制接口（供主线程调用） ----------
    def set_mode(self, mode):
        """mode: 2自动, 3手动, 5调试, 100自动命令"""
        if self.handle is None:
            self.log_signal.emit("[错误] 未连接机器人")
            return
        try:
            x5.set_system_mode(self.handle, mode)
            self.log_signal.emit(f"[模式切换] 成功 -> {mode}")
        except Exception as e:
            self.log_signal.emit(f"[模式切换] 失败: {str(e)}")

    def set_servo(self, on):
        if self.handle is None:
            return
        try:
            x5.enable_servo(self.handle, on)
            self.log_signal.emit(f"[使能] {'上使能' if on else '下使能'} 成功")
        except Exception as e:
            self.log_signal.emit(f"[使能] 失败: {str(e)}")

    def reset_alarm(self):
        if self.handle is None:
            return
        try:
            x5.reset(self.handle)
            self.log_signal.emit("[报警复位] 成功")
        except Exception as e:
            self.log_signal.emit(f"[报警复位] 失败: {str(e)}")

    def emergency_stop(self):
        if self.handle is None:
            return
        try:
            x5.abort(self.handle)
            self.log_signal.emit("[急停] 已触发 abort")
        except Exception as e:
            self.log_signal.emit(f"[急停] 失败: {str(e)}")

    def set_speed(self, speed):
        if self.handle is None:
            return
        try:
            x5.set_speed(self.handle, speed)
            self.log_signal.emit(f"[速度设置] {speed}%")
        except Exception as e:
            self.log_signal.emit(f"[速度设置] 失败: {str(e)}")

    def movj_home(self):
        """测试运动：所有关节回到 0 度"""
        if self.handle is None:
            return
        try:
            target = x5.Joint(0, 0, 0, 0, 0, 0, 0, 0, 0)
            x5.movj(self.handle, target)
            self.log_signal.emit("[运动] 执行 movj 到零点")
        except Exception as e:
            self.log_signal.emit(f"[运动] 失败: {str(e)}")

    # ---------- 线程主循环 ----------
    def run(self):
        while self.running:
            if self.handle is not None:
                self.get_state()
            self.msleep(self.update_interval)
        self.log_signal.emit("[线程] 状态更新循环已退出")


# ---------- 主窗口 ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("X5 机器人通信测试 - 独立模块")
        self.setGeometry(200, 200, 700, 600)

        # 创建工作线程
        self.robot_thread = RobotThread()
        self.robot_thread.connected_signal.connect(self.on_connected)
        self.robot_thread.state_updated.connect(self.on_state_updated)
        self.robot_thread.log_signal.connect(self.append_log)

        self.init_ui()

        # 初始禁用控制按钮
        self._set_controls_enabled(False)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ---------- 连接参数区域 ----------
        form = QFormLayout()
        self.ip_edit = QLineEdit("168.168.40.20")
        form.addRow("控制器 IP:", self.ip_edit)

        conn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        form.addRow("", conn_layout)

        main_layout.addLayout(form)

        # ---------- 状态显示区域 ----------
        status_group = QGroupBox("机器人状态")
        status_layout = QGridLayout()

        self.status_label = QLabel("状态: 未连接")
        self.mode_label = QLabel("模式: --")
        self.enable_label = QLabel("使能: --")
        self.alarm_label = QLabel("报警: --")
        self.speed_label = QLabel("速度: --%")

        status_layout.addWidget(self.status_label, 0, 0)
        status_layout.addWidget(self.mode_label, 0, 1)
        status_layout.addWidget(self.enable_label, 0, 2)
        status_layout.addWidget(self.alarm_label, 0, 3)
        status_layout.addWidget(self.speed_label, 1, 0)

        self.joint_label = QLabel("关节: --")
        self.cart_label = QLabel("笛卡尔: --")
        status_layout.addWidget(self.joint_label, 2, 0, 1, 2)
        status_layout.addWidget(self.cart_label, 3, 0, 1, 2)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # ---------- 控制按钮区域 ----------
        ctrl_group = QGroupBox("控制指令")
        ctrl_layout = QHBoxLayout()

        self.mode_auto_btn = QPushButton("自动模式")
        self.mode_auto_btn.clicked.connect(lambda: self.robot_thread.set_mode(2))

        self.mode_autocmd_btn = QPushButton("自动命令模式")
        self.mode_autocmd_btn.clicked.connect(lambda: self.robot_thread.set_mode(100))

        self.mode_manual_btn = QPushButton("手动模式")
        self.mode_manual_btn.clicked.connect(lambda: self.robot_thread.set_mode(3))

        self.servo_on_btn = QPushButton("上使能")
        self.servo_on_btn.clicked.connect(lambda: self.robot_thread.set_servo(True))

        self.servo_off_btn = QPushButton("下使能")
        self.servo_off_btn.clicked.connect(lambda: self.robot_thread.set_servo(False))

        self.reset_alarm_btn = QPushButton("复位报警")
        self.reset_alarm_btn.clicked.connect(self.robot_thread.reset_alarm)

        ctrl_layout.addWidget(self.mode_auto_btn)
        ctrl_layout.addWidget(self.mode_autocmd_btn)
        ctrl_layout.addWidget(self.mode_manual_btn)
        ctrl_layout.addWidget(self.servo_on_btn)
        ctrl_layout.addWidget(self.servo_off_btn)
        ctrl_layout.addWidget(self.reset_alarm_btn)
        ctrl_group.setLayout(ctrl_layout)
        main_layout.addWidget(ctrl_group)

        # ---------- 速度滑块 ----------
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("全局速度:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(20)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        self.speed_value_label = QLabel("20%")
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_value_label)
        main_layout.addLayout(speed_layout)

        # ---------- 特殊操作（急停 + 归零） ----------
        action_layout = QHBoxLayout()
        self.home_btn = QPushButton("运动到零点 (关节)")
        self.home_btn.clicked.connect(self.robot_thread.movj_home)

        self.emergency_btn = QPushButton("紧急停止 (abort)")
        self.emergency_btn.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        self.emergency_btn.clicked.connect(self.robot_thread.emergency_stop)

        action_layout.addWidget(self.home_btn)
        action_layout.addWidget(self.emergency_btn)
        main_layout.addLayout(action_layout)

        # ---------- 日志区域 ----------
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # 初始化断开按钮状态
        self.disconnect_btn.setEnabled(False)

    def _set_controls_enabled(self, enabled):
        """连接成功后启用所有控制按钮"""
        self.mode_auto_btn.setEnabled(enabled)
        self.mode_autocmd_btn.setEnabled(enabled)
        self.mode_manual_btn.setEnabled(enabled)
        self.servo_on_btn.setEnabled(enabled)
        self.servo_off_btn.setEnabled(enabled)
        self.reset_alarm_btn.setEnabled(enabled)
        self.speed_slider.setEnabled(enabled)
        self.home_btn.setEnabled(enabled)
        self.emergency_btn.setEnabled(enabled)

    # ---------- 槽函数 ----------
    def on_connect(self):
        ip = self.ip_edit.text().strip()
        if not ip:
            self.append_log("[错误] IP 不能为空")
            return
        self.append_log(f"正在连接 {ip} ...")
        self.robot_thread.set_ip(ip)
        self.robot_thread.start()
        # 延时 100ms 执行连接（让线程先启动）
        QTimer.singleShot(100, self.robot_thread.connect_robot)

    def on_disconnect(self):
        self.append_log("正在断开...")
        self.robot_thread.disconnect_robot()
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)
        self.status_label.setText("状态: 未连接")
        self._set_controls_enabled(False)

    def on_connected(self, success):
        if success:
            self.status_label.setText("状态: 已连接")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self._set_controls_enabled(True)
            self.append_log("[状态] 连接成功，开始自动刷新状态")
        else:
            self.status_label.setText("状态: 连接失败")
            self.connect_btn.setEnabled(True)

    def on_state_updated(self, info):
        # 模式
        mode_map = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}
        self.mode_label.setText(f"模式: {mode_map.get(info['mode'], str(info['mode']))}")
        self.enable_label.setText(f"使能: {'ON' if info['enable'] else 'OFF'}")
        self.alarm_label.setText(f"报警: {'有' if info['alarm'] else '无'}")
        self.speed_label.setText(f"速度: {info['speed']}%")

        # 关节
        j = info['joint']
        joint_str = f"J1:{j[0]:.2f} J2:{j[1]:.2f} J3:{j[2]:.2f} J4:{j[3]:.2f} J5:{j[4]:.2f} J6:{j[5]:.2f}"
        self.joint_label.setText(joint_str)

        # 笛卡尔
        p = info['cart']
        cart_str = f"X:{p[0]:.2f} Y:{p[1]:.2f} Z:{p[2]:.2f} A:{p[3]:.2f} B:{p[4]:.2f} C:{p[5]:.2f}"
        self.cart_label.setText(cart_str)

    def on_speed_changed(self, value):
        self.speed_value_label.setText(f"{value}%")
        self.robot_thread.set_speed(value)

    def append_log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self.robot_thread.disconnect_robot()
        self.robot_thread.quit()
        self.robot_thread.wait()
        event.accept()


# ---------- 启动 ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())