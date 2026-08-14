import sys
import socket
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer

import xapi.api as x5
import xapi.api.vision as x5v


# ============================================================
# 工作线程1：与 Vision Master 通信
# ============================================================
class VisionMasterThread(QThread):
    connected_signal = pyqtSignal(bool)
    data_received = pyqtSignal(list)      # 解析后返回点列表
    raw_data_received = pyqtSignal(str)   # 原始数据（用于日志）
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.sock = None
        self.ip = ""
        self.port = 0
        self.running = False
        self.trigger_cmd = "trigger"

    def set_params(self, ip, port, trigger_cmd="trigger"):
        self.ip = ip
        self.port = port
        self.trigger_cmd = trigger_cmd

    def connect_to_server(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip, self.port))
            self.running = True
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[VM] 连接成功 {self.ip}:{self.port}")
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[VM] 连接失败: {str(e)}")
            self.sock = None

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.connected_signal.emit(False)
        self.log_signal.emit("[VM] 已断开")

    def trigger_and_get(self):
        if not self.sock:
            self.log_signal.emit("[VM] 错误: 未连接")
            return
        try:
            self.sock.send(self.trigger_cmd.encode())
            self.log_signal.emit(f"[VM] 发送触发: {self.trigger_cmd}")
            self.sock.settimeout(3.0)
            raw_data = self.sock.recv(4096).decode().strip()
            self.raw_data_received.emit(raw_data)
            self.log_signal.emit(f"[VM] 收到原始数据: {raw_data[:80]}...")

            points = self._parse_data(raw_data)
            if points:
                self.data_received.emit(points)
                self.log_signal.emit(f"[VM] 解析到 {len(points)} 个点")
            else:
                self.log_signal.emit("[VM] 警告: 数据解析为空")

        except socket.timeout:
            self.log_signal.emit("[VM] 超时: 等待响应超时")
        except Exception as e:
            self.log_signal.emit(f"[VM] 错误: {str(e)}")

    def _parse_data(self, raw):
        points = []
        raw = raw.strip()
        if not raw or raw == "Done" or raw == "；":
            return points

        for item in raw.split(";"):
            item = item.strip()
            if not item:
                continue
            parts = item.split(",")
            if len(parts) >= 3:
                try:
                    point = {
                        'x': float(parts[0]),
                        'y': float(parts[1]),
                        'c': float(parts[2]) if len(parts) > 2 else 0.0,
                        'attr': int(parts[3]) if len(parts) > 3 else 0,
                        'id': int(parts[4]) if len(parts) > 4 else 0
                    }
                    points.append(point)
                except ValueError:
                    continue
        return points

    def run(self):
        while True:
            self.msleep(100)
            if not self.running:
                break


# ============================================================
# 工作线程2：机器人 API 调用
# ============================================================
class RobotWorker(QThread):
    state_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)
    calib_result_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.handle = None
        self.ip = ""

    def set_ip(self, ip):
        self.ip = ip

    def connect_robot(self):
        try:
            self.handle = x5.connect(self.ip)
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[机器人] 连接成功，句柄: {self.handle}")
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[机器人] 连接失败: {str(e)}")
            self.handle = None

    def disconnect_robot(self):
        if self.handle is not None:
            try:
                x5.disconnect(self.handle)
            except:
                pass
            self.handle = None
        self.connected_signal.emit(False)
        self.log_signal.emit("[机器人] 已断开")

    def get_state(self):
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

    # ---------- 视觉标定接口 ----------
    def apply_vision_config(self, vision_idx, config, calib_process):
        if self.handle is None:
            return False, "机器人未连接"
        try:
            x5v.vision_set_config(self.handle, vision_idx, config)
            x5v.vision_write_calib_process(self.handle, vision_idx, calib_process)
            return True, "配置写入成功"
        except Exception as e:
            return False, str(e)

    def run_auto_calib(self, vision_idx):
        if self.handle is None:
            return None, "机器人未连接"
        try:
            flag, errors, transformation = x5v.vision_auto_calib(self.handle, vision_idx)
            return {
                'flag': flag,
                'errors': errors,
                'transformation': transformation
            }, None
        except Exception as e:
            return None, str(e)

    def verify_vision(self, vision_idx, pixel_pose, trig_point):
        if self.handle is None:
            return None, "机器人未连接"
        try:
            result_pose = x5v.vision_dynamic_cnvr(self.handle, vision_idx, pixel_pose, trig_point)
            return result_pose, None
        except Exception as e:
            return None, str(e)

    def run(self):
        while True:
            self.msleep(100)
            if not self.isRunning():
                break


# ============================================================
# 主窗口
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("视觉码垛控制系统 - 机器人 + Vision Master")
        self.setGeometry(100, 100, 1000, 900)

        self.robot_worker = RobotWorker()
        self.robot_worker.connected_signal.connect(self.on_robot_connected)
        self.robot_worker.log_signal.connect(self.append_log)

        self.vm_thread = VisionMasterThread()
        self.vm_thread.connected_signal.connect(self.on_vm_connected)
        self.vm_thread.data_received.connect(self.on_vm_data_received)
        self.vm_thread.raw_data_received.connect(self.on_vm_raw_data)
        self.vm_thread.log_signal.connect(self.append_log)

        self.state_timer = QTimer()
        self.state_timer.timeout.connect(self.refresh_state)
        self.state_timer.setInterval(200)

        self.is_robot_connected = False
        self.is_vm_connected = False
        self.base_point = None
        self.latest_vm_points = []

        self.init_ui()
        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ========== 第一行：机器人连接 ==========
        robot_group = QGroupBox("机器人控制")
        robot_layout = QHBoxLayout()
        robot_layout.addWidget(QLabel("IP:"))
        self.robot_ip_edit = QLineEdit("168.168.40.20")
        self.robot_ip_edit.setFixedWidth(150)
        robot_layout.addWidget(self.robot_ip_edit)

        self.robot_connect_btn = QPushButton("连接机器人")
        self.robot_connect_btn.clicked.connect(self.on_robot_connect)
        self.robot_disconnect_btn = QPushButton("断开")
        self.robot_disconnect_btn.clicked.connect(self.on_robot_disconnect)
        self.robot_disconnect_btn.setEnabled(False)
        robot_layout.addWidget(self.robot_connect_btn)
        robot_layout.addWidget(self.robot_disconnect_btn)

        self.robot_status_label = QLabel("⚪ 未连接")
        robot_layout.addWidget(self.robot_status_label)
        robot_layout.addStretch()
        robot_group.setLayout(robot_layout)
        main_layout.addWidget(robot_group)

        # ========== 第二行：Vision Master 连接 ==========
        vm_group = QGroupBox("Vision Master 通信")
        vm_layout = QHBoxLayout()
        vm_layout.addWidget(QLabel("IP:"))
        self.vm_ip_edit = QLineEdit("168.168.40.111")
        self.vm_ip_edit.setFixedWidth(150)
        vm_layout.addWidget(self.vm_ip_edit)

        vm_layout.addWidget(QLabel("端口:"))
        self.vm_port_edit = QLineEdit("4004")
        self.vm_port_edit.setFixedWidth(80)
        vm_layout.addWidget(self.vm_port_edit)

        vm_layout.addWidget(QLabel("触发命令:"))
        self.vm_trigger_edit = QLineEdit("trigger")
        self.vm_trigger_edit.setFixedWidth(100)
        vm_layout.addWidget(self.vm_trigger_edit)

        self.vm_connect_btn = QPushButton("连接VM")
        self.vm_connect_btn.clicked.connect(self.on_vm_connect)
        self.vm_disconnect_btn = QPushButton("断开")
        self.vm_disconnect_btn.clicked.connect(self.on_vm_disconnect)
        self.vm_disconnect_btn.setEnabled(False)
        vm_layout.addWidget(self.vm_connect_btn)
        vm_layout.addWidget(self.vm_disconnect_btn)

        self.vm_status_label = QLabel("⚪ 未连接")
        vm_layout.addWidget(self.vm_status_label)
        vm_layout.addStretch()
        vm_group.setLayout(vm_layout)
        main_layout.addWidget(vm_group)

        # ========== 第三行：状态显示 + 基本控制 ==========
        status_group = QGroupBox("实时状态")
        status_layout = QGridLayout()

        self.mode_label = QLabel("模式: --")
        self.enable_label = QLabel("使能: --")
        self.alarm_label = QLabel("报警: --")
        self.speed_label = QLabel("速度: --%")
        self.joint_label = QLabel("关节: --")
        self.cart_label = QLabel("笛卡尔: --")

        status_layout.addWidget(self.mode_label, 0, 0)
        status_layout.addWidget(self.enable_label, 0, 1)
        status_layout.addWidget(self.alarm_label, 0, 2)
        status_layout.addWidget(self.speed_label, 0, 3)
        status_layout.addWidget(self.joint_label, 1, 0, 1, 2)
        status_layout.addWidget(self.cart_label, 2, 0, 1, 2)

        # ★★★ 修正点：显式创建每个按钮并保存为实例属性 ★★★
        self.mode_auto_btn = QPushButton("自动模式")
        self.mode_auto_btn.clicked.connect(lambda: self.robot_worker.set_mode(2))
        status_layout.addWidget(self.mode_auto_btn, 3, 0)

        self.mode_autocmd_btn = QPushButton("自动命令")
        self.mode_autocmd_btn.clicked.connect(lambda: self.robot_worker.set_mode(100))
        status_layout.addWidget(self.mode_autocmd_btn, 3, 1)

        self.mode_manual_btn = QPushButton("手动模式")
        self.mode_manual_btn.clicked.connect(lambda: self.robot_worker.set_mode(3))
        status_layout.addWidget(self.mode_manual_btn, 3, 2)

        self.servo_on_btn = QPushButton("上使能")
        self.servo_on_btn.clicked.connect(lambda: self.robot_worker.set_servo(True))
        status_layout.addWidget(self.servo_on_btn, 3, 3)

        self.servo_off_btn = QPushButton("下使能")
        self.servo_off_btn.clicked.connect(lambda: self.robot_worker.set_servo(False))
        status_layout.addWidget(self.servo_off_btn, 3, 4)

        self.reset_alarm_btn = QPushButton("复位报警")
        self.reset_alarm_btn.clicked.connect(self.robot_worker.reset_alarm)
        status_layout.addWidget(self.reset_alarm_btn, 3, 5)

        # 速度滑块
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("速度:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(20)
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        self.speed_value_label = QLabel("20%")
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_value_label)
        status_layout.addLayout(speed_layout, 4, 0, 1, 6)

        # 运动测试 + 急停
        self.movj_btn = QPushButton("执行 movj (关节)")
        self.movj_btn.clicked.connect(self.on_movj)
        self.movl_btn = QPushButton("执行 movl (直线)")
        self.movl_btn.clicked.connect(self.on_movl)
        self.emergency_btn = QPushButton("🚨 急停")
        self.emergency_btn.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold;")
        self.emergency_btn.clicked.connect(self.robot_worker.emergency_stop)
        status_layout.addWidget(self.movj_btn, 5, 0)
        status_layout.addWidget(self.movl_btn, 5, 1)
        status_layout.addWidget(self.emergency_btn, 5, 2, 1, 4)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # ========== 第四行：视觉标定区域 ==========
        calib_group = QGroupBox("视觉标定 (SCARA J2 动态)")
        calib_layout = QGridLayout()

        calib_layout.addWidget(QLabel("视觉工艺号:"), 0, 0)
        self.vision_index_combo = QComboBox()
        self.vision_index_combo.addItems([str(i) for i in range(8)])
        self.vision_index_combo.setCurrentIndex(0)
        calib_layout.addWidget(self.vision_index_combo, 0, 1)

        calib_layout.addWidget(QLabel("Mark点间距(mm):"), 0, 2)
        self.mark_distance_edit = QLineEdit("10")
        self.mark_distance_edit.setFixedWidth(60)
        calib_layout.addWidget(self.mark_distance_edit, 0, 3)

        calib_layout.addWidget(QLabel("步长(mm):"), 0, 4)
        self.step_size_edit = QLineEdit("20")
        self.step_size_edit.setFixedWidth(60)
        calib_layout.addWidget(self.step_size_edit, 0, 5)

        calib_layout.addWidget(QLabel("像素U,V:"), 0, 6)
        self.pixel_u_edit = QLineEdit("2592")
        self.pixel_u_edit.setFixedWidth(60)
        self.pixel_v_edit = QLineEdit("2048")
        self.pixel_v_edit.setFixedWidth(60)
        calib_layout.addWidget(self.pixel_u_edit, 0, 7)
        calib_layout.addWidget(self.pixel_v_edit, 0, 8)

        self.base_point_label = QLabel("基准点: 未记录")
        calib_layout.addWidget(self.base_point_label, 1, 0, 1, 4)
        self.record_base_btn = QPushButton("记录基准点")
        self.record_base_btn.clicked.connect(self.on_record_base_point)
        calib_layout.addWidget(self.record_base_btn, 1, 4, 1, 2)

        self.apply_config_btn = QPushButton("① 写入配置")
        self.apply_config_btn.clicked.connect(self.on_apply_config)
        self.start_calib_btn = QPushButton("② 开始标定")
        self.start_calib_btn.clicked.connect(self.on_start_calib)
        self.verify_btn = QPushButton("③ 标定验证")
        self.verify_btn.clicked.connect(self.on_verify)
        self.trigger_vm_btn = QPushButton("📷 触发拍照")
        self.trigger_vm_btn.clicked.connect(self.on_trigger_vm)

        calib_layout.addWidget(self.apply_config_btn, 2, 0)
        calib_layout.addWidget(self.start_calib_btn, 2, 1)
        calib_layout.addWidget(self.verify_btn, 2, 2)
        calib_layout.addWidget(self.trigger_vm_btn, 2, 3)

        self.calib_result_label = QLabel("标定状态: 等待操作")
        calib_layout.addWidget(self.calib_result_label, 3, 0, 1, 6)

        calib_layout.addWidget(QLabel("最近视觉数据:"), 4, 0, 1, 2)
        self.vm_data_display = QTextEdit()
        self.vm_data_display.setReadOnly(True)
        self.vm_data_display.setMaximumHeight(60)
        calib_layout.addWidget(self.vm_data_display, 5, 0, 1, 9)

        self.verify_result_label = QLabel("验证结果: 等待验证")
        calib_layout.addWidget(self.verify_result_label, 6, 0, 1, 6)

        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)

        # ========== 日志区域 ==========
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # 初始禁用
        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    # ============================================================
    # 辅助方法
    # ============================================================
    def _set_robot_controls_enabled(self, enabled):
        for w in [self.movj_btn, self.movl_btn, self.emergency_btn,
                  self.speed_slider, self.mode_auto_btn, self.mode_autocmd_btn,
                  self.mode_manual_btn, self.servo_on_btn, self.servo_off_btn,
                  self.reset_alarm_btn]:
            w.setEnabled(enabled)

    def _set_vm_controls_enabled(self, enabled):
        self.vm_disconnect_btn.setEnabled(enabled)
        self.trigger_vm_btn.setEnabled(enabled)

    def _set_calib_controls_enabled(self, enabled):
        for w in [self.record_base_btn, self.apply_config_btn,
                  self.start_calib_btn, self.verify_btn]:
            w.setEnabled(enabled)

    # ============================================================
    # 机器人相关槽函数
    # ============================================================
    def on_robot_connect(self):
        ip = self.robot_ip_edit.text().strip()
        if not ip:
            self.append_log("[错误] 请输入机器人IP")
            return
        self.append_log(f"正在连接机器人 {ip} ...")
        self.robot_worker.set_ip(ip)
        if not self.robot_worker.isRunning():
            self.robot_worker.start()
        QTimer.singleShot(50, self.robot_worker.connect_robot)

    def on_robot_disconnect(self):
        self.robot_worker.disconnect_robot()
        self.state_timer.stop()
        self.is_robot_connected = False
        self.robot_connect_btn.setEnabled(True)
        self.robot_disconnect_btn.setEnabled(False)
        self.robot_status_label.setText("⚪ 未连接")
        self._set_robot_controls_enabled(False)

    def on_robot_connected(self, success):
        if success:
            self.is_robot_connected = True
            self.robot_status_label.setText("🟢 已连接")
            self.robot_connect_btn.setEnabled(False)
            self.robot_disconnect_btn.setEnabled(True)
            self._set_robot_controls_enabled(True)
            self._set_calib_controls_enabled(True)
            self.append_log("[机器人] 连接成功，开始刷新状态")
            self.state_timer.start()
        else:
            self.robot_status_label.setText("🔴 连接失败")
            self.robot_connect_btn.setEnabled(True)

    def refresh_state(self):
        if not self.is_robot_connected:
            return
        info = self.robot_worker.get_state()
        if info is None:
            return

        mode_map = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}
        self.mode_label.setText(f"模式: {mode_map.get(info['mode'], str(info['mode']))}")
        self.enable_label.setText(f"使能: {'✅ ON' if info['enable'] else '❌ OFF'}")
        self.alarm_label.setText(f"报警: {'⚠️ 有' if info['alarm'] else '✅ 无'}")
        self.speed_label.setText(f"速度: {info['speed']}%")

        j = info['joint']
        self.joint_label.setText(
            f"关节: J1={j[0]:.2f} J2={j[1]:.2f} J3={j[2]:.2f} "
            f"J4={j[3]:.2f} J5={j[4]:.2f} J6={j[5]:.2f}"
        )
        p = info['cart']
        self.cart_label.setText(
            f"笛卡尔: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f} "
            f"A={p[3]:.2f} B={p[4]:.2f} C={p[5]:.2f}"
        )

    def on_speed_changed(self, value):
        self.speed_value_label.setText(f"{value}%")
        self.robot_worker.set_speed(value)

    def on_movj(self):
        self.robot_worker.movj_to(10, 0, 0, 0, 0, 0)

    def on_movl(self):
        self.robot_worker.movl_to(300, 0, 100)

    # ============================================================
    # Vision Master 相关槽函数
    # ============================================================
    def on_vm_connect(self):
        ip = self.vm_ip_edit.text().strip()
        try:
            port = int(self.vm_port_edit.text().strip())
        except ValueError:
            self.append_log("[错误] 端口必须为数字")
            return
        trigger = self.vm_trigger_edit.text().strip()

        self.append_log(f"正在连接 Vision Master {ip}:{port}")
        self.vm_thread.set_params(ip, port, trigger)
        if not self.vm_thread.isRunning():
            self.vm_thread.start()
        QTimer.singleShot(100, self.vm_thread.connect_to_server)

    def on_vm_disconnect(self):
        self.vm_thread.disconnect()
        self.is_vm_connected = False
        self.vm_connect_btn.setEnabled(True)
        self.vm_disconnect_btn.setEnabled(False)
        self.vm_status_label.setText("⚪ 未连接")
        self._set_vm_controls_enabled(False)

    def on_vm_connected(self, success):
        if success:
            self.is_vm_connected = True
            self.vm_status_label.setText("🟢 已连接")
            self.vm_connect_btn.setEnabled(False)
            self.vm_disconnect_btn.setEnabled(True)
            self._set_vm_controls_enabled(True)
            self.append_log("[VM] 连接成功")
        else:
            self.vm_status_label.setText("🔴 连接失败")
            self.vm_connect_btn.setEnabled(True)

    def on_vm_raw_data(self, raw):
        pass

    def on_vm_data_received(self, points):
        self.latest_vm_points = points
        display_text = ""
        for i, p in enumerate(points[:5]):
            display_text += f"点{i+1}: X={p['x']:.2f} Y={p['y']:.2f} C={p['c']:.2f} ATTR={p['attr']} ID={p['id']}  "
        if len(points) > 5:
            display_text += f"... 共 {len(points)} 个点"
        self.vm_data_display.setText(display_text)
        self.append_log(f"[VM] 收到 {len(points)} 个点")

    def on_trigger_vm(self):
        if not self.is_vm_connected:
            self.append_log("[错误] Vision Master 未连接")
            return
        self.append_log("[VM] 触发拍照...")
        self.vm_thread.trigger_and_get()

    # ============================================================
    # 视觉标定相关槽函数
    # ============================================================
    def on_record_base_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        info = self.robot_worker.get_state()
        if info is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        p = info['cart']
        self.base_point = (p[0], p[1], p[2], p[3], p[4], p[5])
        self.base_point_label.setText(f"基准点: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f}")
        self.append_log(f"[基准点] 已记录: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")

    def on_apply_config(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.base_point is None:
            self.append_log("[错误] 请先记录基准点")
            return

        try:
            vision_idx = int(self.vision_index_combo.currentText())

            config = x5v.Vision(
                ip=self.vm_ip_edit.text().strip().encode(),
                port=int(self.vm_port_edit.text().strip()),
                communication_protocol=0,
                auto_start=False,
                data_format=1,
                invert_c_value=False,
                camera_mount_type=2,        # 2: 动态非末端 (SCARA J2)
                uf_index=0,
                calibration_object=1,
                trigger_type=0,
                trigger_do=255,
                trigger_command=self.vm_trigger_edit.text().strip().encode()
            )

            pixel_u = int(self.pixel_u_edit.text())
            pixel_v = int(self.pixel_v_edit.text())
            mark_dist = int(self.mark_distance_edit.text())
            step = int(self.step_size_edit.text())

            base_pose = x5.Pose(
                self.base_point[0], self.base_point[1], self.base_point[2],
                self.base_point[3], self.base_point[4], self.base_point[5],
                0, 0, 0
            )
            base_point = x5.Point(pose=base_pose, uf=0, tf=0, cfg=(0, 0, 0, 1))

            calib_process = x5v.VisionCalibProcess()
            calib_process.calibration_type = 1
            calib_process.point_count = 0
            calib_process.is_workplane_parallel = True

            calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
                tf_index=0,
                pixel_shift=[pixel_u, pixel_v],
                error=0.0,
                base_point=base_point,
                mark_distance=mark_dist,
                step_size=step
            )

            success, msg = self.robot_worker.apply_vision_config(
                vision_idx, config, calib_process
            )
            if success:
                self.append_log(f"[配置] {msg}")
                self.calib_result_label.setText("标定状态: 配置已写入")
            else:
                self.append_log(f"[配置] 失败: {msg}")
                self.calib_result_label.setText(f"标定状态: 配置失败")

        except Exception as e:
            self.append_log(f"[配置] 异常: {str(e)}")
            self.calib_result_label.setText("标定状态: 配置异常")

    def on_start_calib(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        vision_idx = int(self.vision_index_combo.currentText())
        self.append_log("[标定] 开始自动标定，请确保周围安全...")
        self.calib_result_label.setText("标定状态: 进行中...")

        result, err = self.robot_worker.run_auto_calib(vision_idx)

        if err:
            self.append_log(f"[标定] 异常: {err}")
            self.calib_result_label.setText("标定状态: ❌ 异常")
            return

        if result is None:
            self.append_log("[标定] 失败")
            self.calib_result_label.setText("标定状态: ❌ 失败")
            return

        flag = result['flag']
        errors = result['errors']

        if flag == 1:
            self.append_log(f"[标定] ✅ 成功！误差: X平均={errors[0]:.3f}mm, Y平均={errors[1]:.3f}mm")
            self.calib_result_label.setText(f"标定状态: ✅ 成功 (误差 {errors[0]:.3f}mm)")
            x5v.vision_set_calib_par(self.robot_worker.handle, vision_idx, result['transformation'])
            self.append_log("[标定] 标定参数已应用")
        elif flag == -1:
            self.append_log("[标定] ⚠️ 成功但误差超标 (>5mm)")
            self.calib_result_label.setText("标定状态: ⚠️ 误差超标")
        else:
            self.append_log("[标定] ❌ 失败，请检查标定板和 ATTR 识别")
            self.calib_result_label.setText("标定状态: ❌ 失败")

    def on_verify(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        if not self.latest_vm_points:
            self.append_log("[错误] 请先触发拍照获取视觉数据")
            return

        p = self.latest_vm_points[0]
        vision_idx = int(self.vision_index_combo.currentText())

        info = self.robot_worker.get_state()
        if info is None:
            self.append_log("[错误] 获取机器人位置失败")
            return

        cart = info['cart']
        trig_pose = x5.Pose(cart[0], cart[1], cart[2], cart[3], cart[4], cart[5], 0, 0, 0)
        trig_point = x5.Point(pose=trig_pose, uf=0, tf=0, cfg=(0, 0, 0, 1))

        pixel_pose = x5.Pose(p['x'], p['y'], 0, 0, 0, p['c'], 0, 0, 0)

        self.append_log(f"[验证] 像素点: X={p['x']:.2f} Y={p['y']:.2f} C={p['c']:.2f} ATTR={p['attr']}")

        result_pose, err = self.robot_worker.verify_vision(vision_idx, pixel_pose, trig_point)

        if err:
            self.append_log(f"[验证] 失败: {err}")
            self.verify_result_label.setText(f"验证结果: ❌ {err}")
            return

        self.verify_result_label.setText(
            f"验证结果: 机器人坐标 X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}"
        )
        self.append_log(f"[验证] 转换后: X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}")

    # ============================================================
    # 日志
    # ============================================================
    def append_log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")

    def closeEvent(self, event):
        self.state_timer.stop()
        self.vm_thread.disconnect()
        self.vm_thread.quit()
        self.vm_thread.wait()
        self.robot_worker.disconnect_robot()
        self.robot_worker.quit()
        self.robot_worker.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())