import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer

import xapi.api as x5


# ---------- 工作线程：只负责执行 API 调用（不包含循环） ----------
class RobotWorker(QThread):
    """专门执行机器人 API 调用的工作线程，避免阻塞 UI"""
    state_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.handle = None
        self.ip = ""
    
    def set_ip(self, ip):
        self.ip = ip
    
    def connect_robot(self):
        """连接机器人"""
        try:
            self.handle = x5.connect(self.ip)
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[机器人] 连接成功，句柄: {self.handle}")
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[机器人] 连接失败: {str(e)}")
            self.handle = None
    
    def disconnect_robot(self):
        """断开机器人"""
        if self.handle is not None:
            try:
                x5.disconnect(self.handle)
            except:
                pass
            self.handle = None
        self.connected_signal.emit(False)
        self.log_signal.emit("[机器人] 已断开")
    
    def get_state(self):
        """获取一次状态"""
        if self.handle is None:
            return None
        try:
            state = x5.get_system_state(self.handle)
            speed = x5.get_speed(self.handle)
            joint = x5.get_cjoint(self.handle)
            point = x5.get_cpoint(self.handle)
            
            info = {
                'mode': state.mode,
                'enable': state.enable,
                'alarm': state.alarm,
                'remote': state.remote,
                'run': state.run,
                'in_pos': state.in_pos,
                'speed': speed,
                'joint': (joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6),
                'cart': (point.pose.x, point.pose.y, point.pose.z,
                         point.pose.a, point.pose.b, point.pose.c)
            }
            return info
        except Exception as e:
            self.log_signal.emit(f"[状态获取异常] {str(e)}")
            return None
    
    # ---------- 控制接口 ----------
    def set_mode(self, mode):
        if self.handle is None:
            return
        try:
            x5.set_system_mode(self.handle, mode)
            names = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}
            self.log_signal.emit(f"[模式] -> {names.get(mode, str(mode))}")
        except Exception as e:
            self.log_signal.emit(f"[模式] 失败: {str(e)}")
    
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
            self.log_signal.emit("[急停] 已触发")
        except Exception as e:
            self.log_signal.emit(f"[急停] 失败: {str(e)}")
    
    def set_speed(self, speed):
        if self.handle is None:
            return
        try:
            x5.set_speed(self.handle, speed)
            self.log_signal.emit(f"[速度] {speed}%")
        except Exception as e:
            self.log_signal.emit(f"[速度] 失败: {str(e)}")
    
    def movj_to(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        if self.handle is None:
            return
        try:
            target = x5.Joint(j1, j2, j3, j4, j5, j6, 0, 0, 0)
            x5.movj(self.handle, target)
            self.log_signal.emit(f"[运动] movj 到 ({j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}, {j5:.1f}, {j6:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[运动] 失败: {str(e)}")
    
    def movl_to(self, x, y, z, a=0, b=0, c=0):
        if self.handle is None:
            return
        try:
            pose = x5.Pose(x, y, z, a, b, c, 0, 0, 0)
            point = x5.Point(pose=pose, uf=0, tf=0, cfg=(0, 0, 0, 1))
            x5.movl(self.handle, point)
            self.log_signal.emit(f"[运动] movl 到 ({x:.1f}, {y:.1f}, {z:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[运动] 失败: {str(e)}")


# ---------- 主窗口 ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("X5 机器人控制 - 稳定版")
        self.setGeometry(200, 200, 850, 750)
        
        # 创建工作线程
        self.worker = RobotWorker()
        self.worker.connected_signal.connect(self.on_connected)
        self.worker.log_signal.connect(self.append_log)
        
        self.init_ui()
        self._set_controls_enabled(False)
        
        # ★★★ 核心：用 QTimer 定期获取状态（在主线程中触发）★★★
        self.state_timer = QTimer()
        self.state_timer.timeout.connect(self.refresh_state)
        self.state_timer.setInterval(200)  # 200ms 刷新一次
        
        # 连接状态标记
        self.is_connected = False
    
    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        
        # ---------- 连接区域 ----------
        conn_group = QGroupBox("连接设置")
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("IP:"))
        self.ip_edit = QLineEdit("168.168.40.20")
        self.ip_edit.setFixedWidth(150)
        conn_layout.addWidget(self.ip_edit)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.disconnect_btn.setEnabled(False)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)
        
        # ---------- 状态显示 ----------
        status_group = QGroupBox("实时状态")
        status_layout = QGridLayout()
        self.status_label = QLabel("⚪ 未连接")
        self.status_label.setStyleSheet("font-weight: bold;")
        self.mode_label = QLabel("模式: --")
        self.enable_label = QLabel("使能: --")
        self.alarm_label = QLabel("报警: --")
        self.speed_label = QLabel("速度: --%")
        self.joint_label = QLabel("关节: --")
        self.cart_label = QLabel("笛卡尔: --")
        
        status_layout.addWidget(self.status_label, 0, 0)
        status_layout.addWidget(self.mode_label, 0, 1)
        status_layout.addWidget(self.enable_label, 0, 2)
        status_layout.addWidget(self.alarm_label, 0, 3)
        status_layout.addWidget(self.speed_label, 1, 0)
        status_layout.addWidget(self.joint_label, 2, 0, 1, 2)
        status_layout.addWidget(self.cart_label, 3, 0, 1, 2)
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # ---------- 模式 & 使能 ----------
        ctrl_group = QGroupBox("模式 & 使能")
        ctrl_layout = QHBoxLayout()
        self.mode_auto_btn = QPushButton("自动模式")
        self.mode_auto_btn.clicked.connect(lambda: self.worker.set_mode(2))
        self.mode_autocmd_btn = QPushButton("自动命令")
        self.mode_autocmd_btn.clicked.connect(lambda: self.worker.set_mode(100))
        self.mode_manual_btn = QPushButton("手动模式")
        self.mode_manual_btn.clicked.connect(lambda: self.worker.set_mode(3))
        self.servo_on_btn = QPushButton("上使能")
        self.servo_on_btn.clicked.connect(lambda: self.worker.set_servo(True))
        self.servo_off_btn = QPushButton("下使能")
        self.servo_off_btn.clicked.connect(lambda: self.worker.set_servo(False))
        self.reset_alarm_btn = QPushButton("复位报警")
        self.reset_alarm_btn.clicked.connect(self.worker.reset_alarm)
        
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
        
        # ---------- 运动控制 ----------
        move_group = QGroupBox("运动控制 (测试用)")
        move_layout = QGridLayout()
        
        # 关节输入
        move_layout.addWidget(QLabel("关节目标:"), 0, 0, 1, 6)
        joints = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        self.joint_edits = []
        for i, name in enumerate(joints):
            col = i % 3
            row = 1 + i // 3
            move_layout.addWidget(QLabel(name), row, col*2)
            edit = QLineEdit("0")
            edit.setFixedWidth(50)
            self.joint_edits.append(edit)
            move_layout.addWidget(edit, row, col*2 + 1)
        
        self.movj_btn = QPushButton("执行 movj (关节)")
        self.movj_btn.clicked.connect(self.on_movj)
        move_layout.addWidget(self.movj_btn, 3, 0, 1, 3)
        
        # 笛卡尔输入
        move_layout.addWidget(QLabel("笛卡尔目标:"), 4, 0, 1, 6)
        self.x_edit = QLineEdit("300")
        self.y_edit = QLineEdit("0")
        self.z_edit = QLineEdit("100")
        self.x_edit.setFixedWidth(60)
        self.y_edit.setFixedWidth(60)
        self.z_edit.setFixedWidth(60)
        move_layout.addWidget(QLabel("X"), 5, 0)
        move_layout.addWidget(self.x_edit, 5, 1)
        move_layout.addWidget(QLabel("Y"), 5, 2)
        move_layout.addWidget(self.y_edit, 5, 3)
        move_layout.addWidget(QLabel("Z"), 5, 4)
        move_layout.addWidget(self.z_edit, 5, 5)
        
        self.movl_btn = QPushButton("执行 movl (直线)")
        self.movl_btn.clicked.connect(self.on_movl)
        move_layout.addWidget(self.movl_btn, 6, 0, 1, 3)
        
        # 急停
        self.emergency_btn = QPushButton("🚨 紧急停止 (abort)")
        self.emergency_btn.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; font-size: 14px;")
        self.emergency_btn.clicked.connect(self.worker.emergency_stop)
        move_layout.addWidget(self.emergency_btn, 7, 0, 1, 4)
        
        move_group.setLayout(move_layout)
        main_layout.addWidget(move_group)
        
        # ---------- 日志 ----------
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
    
    def _set_controls_enabled(self, enabled):
        for w in [self.mode_auto_btn, self.mode_autocmd_btn, self.mode_manual_btn,
                  self.servo_on_btn, self.servo_off_btn, self.reset_alarm_btn,
                  self.speed_slider, self.movj_btn, self.movl_btn, self.emergency_btn]:
            w.setEnabled(enabled)
    
    # ---------- 核心：刷新状态 ----------
    def refresh_state(self):
        """由 QTimer 定时调用，获取状态并更新界面"""
        if not self.is_connected:
            return
        info = self.worker.get_state()
        if info is None:
            return
        
        mode_map = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}
        self.mode_label.setText(f"模式: {mode_map.get(info['mode'], str(info['mode']))}")
        self.enable_label.setText(f"使能: {'✅ ON' if info['enable'] else '❌ OFF'}")
        self.alarm_label.setText(f"报警: {'⚠️ 有' if info['alarm'] else '✅ 无'}")
        self.speed_label.setText(f"速度: {info['speed']}%")
        
        j = info['joint']
        self.joint_label.setText(
            f"关节: J1={j[0]:.2f}  J2={j[1]:.2f}  J3={j[2]:.2f}  "
            f"J4={j[3]:.2f}  J5={j[4]:.2f}  J6={j[5]:.2f}"
        )
        
        p = info['cart']
        self.cart_label.setText(
            f"笛卡尔: X={p[0]:.2f}  Y={p[1]:.2f}  Z={p[2]:.2f}  "
            f"A={p[3]:.2f}  B={p[4]:.2f}  C={p[5]:.2f}"
        )
    
    # ---------- 槽函数 ----------
    def on_connect(self):
        ip = self.ip_edit.text().strip()
        if not ip:
            self.append_log("[错误] 请输入 IP")
            return
        self.append_log(f"正在连接 {ip} ...")
        self.worker.set_ip(ip)
        # 启动工作线程（如果还没启动）
        if not self.worker.isRunning():
            self.worker.start()
        # 在子线程中执行连接
        QTimer.singleShot(50, self.worker.connect_robot)
    
    def on_disconnect(self):
        self.worker.disconnect_robot()
        self.state_timer.stop()
        self.is_connected = False
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)
        self.status_label.setText("⚪ 未连接")
        self._set_controls_enabled(False)
        # 清空状态显示
        self.mode_label.setText("模式: --")
        self.enable_label.setText("使能: --")
        self.alarm_label.setText("报警: --")
        self.speed_label.setText("速度: --%")
        self.joint_label.setText("关节: --")
        self.cart_label.setText("笛卡尔: --")
    
    def on_connected(self, success):
        if success:
            self.is_connected = True
            self.status_label.setText("🟢 已连接")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self._set_controls_enabled(True)
            self.append_log("[状态] 连接成功，开始自动刷新状态")
            # ★★★ 启动定时器刷新状态 ★★★
            self.state_timer.start()
        else:
            self.status_label.setText("🔴 连接失败")
            self.status_label.setStyleSheet("font-weight: bold; color: red;")
            self.connect_btn.setEnabled(True)
    
    def on_speed_changed(self, value):
        self.speed_value_label.setText(f"{value}%")
        self.worker.set_speed(value)
    
    def on_movj(self):
        try:
            vals = [float(edit.text()) for edit in self.joint_edits]
            self.worker.movj_to(*vals)
        except ValueError:
            self.append_log("[错误] 关节值必须为数字")
    
    def on_movl(self):
        try:
            x = float(self.x_edit.text())
            y = float(self.y_edit.text())
            z = float(self.z_edit.text())
            self.worker.movl_to(x, y, z)
        except ValueError:
            self.append_log("[错误] 笛卡尔值必须为数字")
    
    def append_log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
    
    def closeEvent(self, event):
        self.state_timer.stop()
        self.worker.disconnect_robot()
        self.worker.quit()
        self.worker.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())