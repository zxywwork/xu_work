import sys
import socket
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont

import xapi.api as x5
import xapi.api.vision as x5v


# ============================================================
# 工作线程1：与 Vision Master 通信
# ============================================================
class VisionMasterThread(QThread):
    connected_signal = pyqtSignal(bool)
    data_received = pyqtSignal(list)
    raw_data_received = pyqtSignal(str)
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
            self.log_signal.emit(f"[VM] 收到原始数据长度: {len(raw_data)} 字符")

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
            self.log_signal.emit(f"[配置] Vision {vision_idx} 参数写入成功")
            x5v.vision_write_calib_process(self.handle, vision_idx, calib_process)
            self.log_signal.emit(f"[配置] Vision {vision_idx} 标定参数写入成功")
            return True, "配置写入成功"
        except Exception as e:
            return False, str(e)

    def run_auto_calib(self, vision_idx):
        if self.handle is None:
            return None, "机器人未连接"
        try:
            self.log_signal.emit("[标定] 正在启动自动标定流程...")
            self.log_signal.emit("[标定] 请确保机器人周围安全，标定板在视野内")
            self.log_signal.emit("[标定] 机器人将自动移动 J2 轴完成标定")

            flag, errors, transformation = x5v.vision_auto_calib(self.handle, vision_idx)

            self.log_signal.emit(f"[标定] 返回标志 flag={flag}")

            if flag == 1:
                self.log_signal.emit(f"[标定] ✅ X方向平均误差: {errors[0]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ Y方向平均误差: {errors[1]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ X方向最大误差: {errors[2]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ Y方向最大误差: {errors[3]:.4f} mm")
            elif flag == -1:
                self.log_signal.emit("[标定] ⚠️ 标定完成但误差超标 (建议 >5mm 重新标定)")
            else:
                self.log_signal.emit("[标定] ❌ 标定失败，请检查:")
                self.log_signal.emit("  1. 标定板是否在相机视野内")
                self.log_signal.emit("  2. Vision Master 是否返回正确的 ATTR (0=圆形,1=三角形,2=四边形,3=五边形)")
                self.log_signal.emit("  3. 数据格式是否为: X,Y,C,ATTR,ID;")

            return {
                'flag': flag,
                'errors': errors,
                'transformation': transformation
            }, None
        except Exception as e:
            self.log_signal.emit(f"[标定] 异常: {str(e)}")
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
        self.setWindowTitle("视觉码垛控制系统 - SCARA J2 动态标定")
        self.setGeometry(50, 50, 1200, 980)

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
        self.calib_started = False

        self.init_ui()
        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ============================================================
        # 第一行：机器人连接
        # ============================================================
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

        # ============================================================
        # 第二行：Vision Master 通信（与文档一致）
        # ============================================================
        vm_group = QGroupBox("视觉通信 (Vision Master)")
        vm_layout = QGridLayout()

        # 协议类型
        vm_layout.addWidget(QLabel("协议类型:"), 0, 0)
        self.vm_protocol_combo = QComboBox()
        self.vm_protocol_combo.addItems(["机器人TCPClient", "机器人TCPServer"])
        self.vm_protocol_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_protocol_combo, 0, 1)

        # IP 地址
        vm_layout.addWidget(QLabel("IP地址:"), 0, 2)
        self.vm_ip_edit = QLineEdit("168.168.40.111")
        self.vm_ip_edit.setFixedWidth(140)
        vm_layout.addWidget(self.vm_ip_edit, 0, 3)

        # 端口号
        vm_layout.addWidget(QLabel("端口号:"), 0, 4)
        self.vm_port_edit = QLineEdit("4004")
        self.vm_port_edit.setFixedWidth(80)
        vm_layout.addWidget(self.vm_port_edit, 0, 5)

        # 触发方式
        vm_layout.addWidget(QLabel("触发方式:"), 1, 0)
        self.vm_trigger_type_combo = QComboBox()
        self.vm_trigger_type_combo.addItems(["网络触发", "IO触发"])
        self.vm_trigger_type_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_trigger_type_combo, 1, 1)

        # 触发字符
        vm_layout.addWidget(QLabel("触发字符:"), 1, 2)
        self.vm_trigger_edit = QLineEdit("trigger")
        self.vm_trigger_edit.setFixedWidth(140)
        vm_layout.addWidget(self.vm_trigger_edit, 1, 3)

        # 接收格式
        vm_layout.addWidget(QLabel("接收格式:"), 1, 4)
        self.vm_format_combo = QComboBox()
        self.vm_format_combo.addItems([str(i) for i in range(10)])
        self.vm_format_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_format_combo, 1, 5)

        # C值取反
        self.vm_c_invert_check = QCheckBox("C值取反")
        vm_layout.addWidget(self.vm_c_invert_check, 2, 0)

        # 视觉坐标系
        vm_layout.addWidget(QLabel("视觉坐标系:"), 2, 1)
        self.vm_uf_combo = QComboBox()
        self.vm_uf_combo.addItems([f"UF{i}" for i in range(17)])
        self.vm_uf_combo.setCurrentIndex(15)
        vm_layout.addWidget(self.vm_uf_combo, 2, 2)

        # 安装类型
        vm_layout.addWidget(QLabel("安装类型:"), 2, 3)
        self.vm_mount_combo = QComboBox()
        self.vm_mount_combo.addItems(["静态正装", "静态倒装", "动态J2", "动态J4", "动态J6"])
        self.vm_mount_combo.setCurrentIndex(2)
        vm_layout.addWidget(self.vm_mount_combo, 2, 4)

        # 超时时间
        vm_layout.addWidget(QLabel("超时时间:"), 2, 5)
        self.vm_timeout_edit = QLineEdit("3.0")
        self.vm_timeout_edit.setFixedWidth(60)
        vm_layout.addWidget(self.vm_timeout_edit, 2, 6)

        # 标定选择
        vm_layout.addWidget(QLabel("标定选择:"), 3, 0)
        self.vm_calib_object_combo = QComboBox()
        self.vm_calib_object_combo.addItems(["视觉软件", "机器人"])
        self.vm_calib_object_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_calib_object_combo, 3, 1)

        self.vm_calib_type_combo = QComboBox()
        self.vm_calib_type_combo.addItems(["手动标定", "自动标定"])
        self.vm_calib_type_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_calib_type_combo, 3, 2)

        self.vm_calib_points_combo = QComboBox()
        self.vm_calib_points_combo.addItems(["9点", "16点", "25点"])
        self.vm_calib_points_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_calib_points_combo, 3, 3)

        # 连接/断开/触发按钮
        self.vm_connect_btn = QPushButton("连接")
        self.vm_connect_btn.clicked.connect(self.on_vm_connect)
        self.vm_disconnect_btn = QPushButton("关闭")
        self.vm_disconnect_btn.clicked.connect(self.on_vm_disconnect)
        self.vm_disconnect_btn.setEnabled(False)
        self.vm_trigger_btn = QPushButton("📷 触发拍照")
        self.vm_trigger_btn.clicked.connect(self.on_trigger_vm)
        self.vm_trigger_btn.setEnabled(False)

        vm_layout.addWidget(self.vm_connect_btn, 3, 4)
        vm_layout.addWidget(self.vm_disconnect_btn, 3, 5)
        vm_layout.addWidget(self.vm_trigger_btn, 3, 6)

        # 状态指示
        self.vm_status_label = QLabel("⚪ 未连接")
        vm_layout.addWidget(self.vm_status_label, 4, 0, 1, 3)

        # 接收格式说明
        vm_layout.addWidget(QLabel("接收格式1: XX,YY,CC; XX,YY,CC,ID; XX,YY,CC,ATTR,ID;"), 5, 0, 1, 7)

        vm_group.setLayout(vm_layout)
        main_layout.addWidget(vm_group)

        # ============================================================
        # 第三行：实时状态 + 基本控制
        # ============================================================
        status_group = QGroupBox("机器人实时状态")
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

        # 控制按钮
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
        self.movj_btn = QPushButton("执行 movj")
        self.movj_btn.clicked.connect(self.on_movj)
        self.movl_btn = QPushButton("执行 movl")
        self.movl_btn.clicked.connect(self.on_movl)
        self.emergency_btn = QPushButton("🚨 急停")
        self.emergency_btn.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; font-size: 14px;")
        self.emergency_btn.clicked.connect(self.robot_worker.emergency_stop)
        status_layout.addWidget(self.movj_btn, 5, 0)
        status_layout.addWidget(self.movl_btn, 5, 1)
        status_layout.addWidget(self.emergency_btn, 5, 2, 1, 4)

        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)

        # ============================================================
        # 第四行：视觉标定
        # ============================================================
        calib_group = QGroupBox("视觉标定 (SCARA J2 动态)")
        calib_layout = QGridLayout()

        # 视觉工艺号
        calib_layout.addWidget(QLabel("视觉工艺号:"), 0, 0)
        self.vision_index_combo = QComboBox()
        self.vision_index_combo.addItems([f"VISION {i}" for i in range(8)])
        self.vision_index_combo.setCurrentIndex(0)
        calib_layout.addWidget(self.vision_index_combo, 0, 1)

        # 标定板参数
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

        # 基准点
        self.base_point_label = QLabel("基准点: 未记录")
        calib_layout.addWidget(self.base_point_label, 1, 0, 1, 4)
        self.record_base_btn = QPushButton("记录基准点")
        self.record_base_btn.clicked.connect(self.on_record_base_point)
        calib_layout.addWidget(self.record_base_btn, 1, 4, 1, 2)

        # 操作按钮
        self.apply_config_btn = QPushButton("① 写入配置")
        self.apply_config_btn.clicked.connect(self.on_apply_config)
        self.start_calib_btn = QPushButton("② 开始标定")
        self.start_calib_btn.clicked.connect(self.on_start_calib)
        self.verify_btn = QPushButton("③ 标定验证")
        self.verify_btn.clicked.connect(self.on_verify)

        calib_layout.addWidget(self.apply_config_btn, 2, 0)
        calib_layout.addWidget(self.start_calib_btn, 2, 1)
        calib_layout.addWidget(self.verify_btn, 2, 2)

        # 标定结果
        self.calib_result_label = QLabel("标定状态: 等待操作")
        calib_layout.addWidget(self.calib_result_label, 3, 0, 1, 6)

        # 视觉数据展示（完整列表）
        calib_layout.addWidget(QLabel("视觉数据 (完整列表):"), 4, 0, 1, 2)
        self.vm_data_list = QListWidget()
        self.vm_data_list.setMaximumHeight(120)
        calib_layout.addWidget(self.vm_data_list, 5, 0, 1, 9)

        # 验证结果
        self.verify_result_label = QLabel("验证结果: 等待验证")
        calib_layout.addWidget(self.verify_result_label, 6, 0, 1, 6)

        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)

        # ============================================================
        # 日志区域（带清空按钮）
        # ============================================================
        log_group = QGroupBox("日志 (标定进度详情)")
        log_layout = QVBoxLayout()

        # 日志工具栏（清空按钮）
        log_toolbar = QHBoxLayout()
        log_toolbar.addWidget(QLabel("日志记录:"))
        log_toolbar.addStretch()
        self.clear_log_btn = QPushButton("🗑️ 清空日志")
        self.clear_log_btn.clicked.connect(self.clear_log)
        self.clear_log_btn.setFixedWidth(120)
        log_toolbar.addWidget(self.clear_log_btn)
        log_layout.addLayout(log_toolbar)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        font = QFont("Consolas", 9)
        self.log_text.setFont(font)
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
        self.vm_trigger_btn.setEnabled(enabled)

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
        self.vm_trigger_btn.setEnabled(False)
        self.vm_status_label.setText("⚪ 未连接")

    def on_vm_connected(self, success):
        if success:
            self.is_vm_connected = True
            self.vm_status_label.setText("🟢 已连接")
            self.vm_connect_btn.setEnabled(False)
            self.vm_disconnect_btn.setEnabled(True)
            self.vm_trigger_btn.setEnabled(True)
            self.append_log("[VM] 连接成功")
        else:
            self.vm_status_label.setText("🔴 连接失败")
            self.vm_connect_btn.setEnabled(True)

    def on_vm_raw_data(self, raw):
        self.append_log(f"[VM] 原始数据: {raw[:120]}...")

    def on_vm_data_received(self, points):
        self.latest_vm_points = points
        self.vm_data_list.clear()
        for i, p in enumerate(points):
            item_text = (f"{i+1:2d}. X={p['x']:8.3f}  Y={p['y']:8.3f}  "
                        f"C={p['c']:7.3f}  ATTR={p['attr']}  ID={p['id']}")
            self.vm_data_list.addItem(item_text)
        self.append_log(f"[VM] 收到 {len(points)} 个点 (完整显示)")

    def on_trigger_vm(self):
        if not self.is_vm_connected:
            self.append_log("[错误] Vision Master 未连接")
            return
        self.append_log("[VM] 触发拍照...")
        self.vm_data_list.clear()
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

    def _get_mount_type_code(self):
        mapping = {
            "静态正装": 0,
            "静态倒装": 1,
            "动态J2": 2,
            "动态J4": 3,
            "动态J6": 4
        }
        return mapping.get(self.vm_mount_combo.currentText(), 0)

    def on_apply_config(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.base_point is None:
            self.append_log("[错误] 请先记录基准点")
            return

        try:
            vision_idx = int(self.vision_index_combo.currentText().split()[-1])

            protocol = 0 if self.vm_protocol_combo.currentIndex() == 0 else 1
            trigger_type = 0 if self.vm_trigger_type_combo.currentIndex() == 0 else 1
            calib_object = 0 if self.vm_calib_object_combo.currentIndex() == 0 else 1
            calib_type = 0 if self.vm_calib_type_combo.currentIndex() == 0 else 1
            point_count = self.vm_calib_points_combo.currentIndex()
            mount_type = self._get_mount_type_code()
            uf_index = self.vm_uf_combo.currentIndex()
            invert_c = self.vm_c_invert_check.isChecked()
            timeout = float(self.vm_timeout_edit.text().strip())

            self.append_log(f"[配置] 视觉工艺: {vision_idx}")
            self.append_log(f"[配置] 协议: {self.vm_protocol_combo.currentText()}")
            self.append_log(f"[配置] 安装类型: {self.vm_mount_combo.currentText()} (code={mount_type})")
            self.append_log(f"[配置] 视觉坐标系: UF{uf_index}")
            self.append_log(f"[配置] 标定: {self.vm_calib_type_combo.currentText()} {self.vm_calib_points_combo.currentText()}")

            config = x5v.Vision(
                ip=self.vm_ip_edit.text().strip().encode(),
                port=int(self.vm_port_edit.text().strip()),
                communication_protocol=protocol,
                auto_start=False,
                data_format=int(self.vm_format_combo.currentText()),
                invert_c_value=invert_c,
                camera_mount_type=mount_type,
                uf_index=uf_index,
                calibration_object=calib_object,
                trigger_type=trigger_type,
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
            base_point = x5.Point(pose=base_pose, uf=uf_index, tf=0, cfg=(0, 0, 0, 1))

            calib_process = x5v.VisionCalibProcess()
            calib_process.calibration_type = calib_type
            calib_process.point_count = point_count
            calib_process.is_workplane_parallel = True

            if mount_type in [2, 3, 4]:
                calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
                    tf_index=0,
                    pixel_shift=[pixel_u, pixel_v],
                    error=0.0,
                    base_point=base_point,
                    mark_distance=mark_dist,
                    step_size=step
                )
                self.append_log("[配置] 使用动态标定参数 (J2/J4/J6)")
            else:
                self.append_log("[警告] 静态标定需要额外配置 P1/P2 点")

            success, msg = self.robot_worker.apply_vision_config(vision_idx, config, calib_process)
            if success:
                self.append_log(f"[配置] ✅ {msg}")
                self.calib_result_label.setText("标定状态: 配置已写入 ✅")
            else:
                self.append_log(f"[配置] ❌ 失败: {msg}")
                self.calib_result_label.setText(f"标定状态: 配置失败 ❌")

        except Exception as e:
            self.append_log(f"[配置] 异常: {str(e)}")
            self.calib_result_label.setText("标定状态: 配置异常 ❌")

    # ============================================================
    # ★★★ 修正：开始标定（自动切换到自动模式）★★★
    # ============================================================
    def on_start_calib(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        info = self.robot_worker.get_state()
        if info is None:
            self.append_log("[错误] 获取机器人状态失败")
            return

        if info['enable'] == 0:
            self.append_log("[警告] 机器人未上使能，请先点击「上使能」")
            return

        # 标定需要自动模式 (mode=2)
        if info['mode'] != 2:
            self.append_log("[标定] 当前模式不是「自动模式」，正在自动切换...")
            self.robot_worker.set_mode(2)
            QTimer.singleShot(800, self._do_start_calib)
            return
        else:
            self._do_start_calib()

    def _do_start_calib(self):
        info = self.robot_worker.get_state()
        if info is None:
            self.append_log("[错误] 获取机器人状态失败")
            return

        if info['mode'] != 2:
            self.append_log("[错误] 未能切换到自动模式，请手动切换后重试")
            self.calib_result_label.setText("标定状态: ❌ 模式切换失败")
            return

        if info['run'] != 3:
            self.append_log("[标定] 机器人正在运行中，执行停止...")
            self.robot_worker.emergency_stop()
            time.sleep(0.3)

        vision_idx = int(self.vision_index_combo.currentText().split()[-1])

        self.append_log("=" * 60)
        self.append_log("[标定] 开始自动标定流程...")
        self.append_log("[标定] ✅ 当前模式: 自动 (mode=2)")
        self.append_log("[标定] ✅ 使能状态: ON")
        self.append_log("[标定] 请确保:")
        self.append_log("  1. 标定板在相机视野内")
        self.append_log("  2. 机器人周围无障碍物")
        self.append_log("  3. Vision Master 已连接并返回正确的 ATTR")
        self.append_log("=" * 60)

        self.calib_result_label.setText("标定状态: 进行中... ⏳")
        self.calib_result_label.setStyleSheet("color: orange; font-weight: bold;")

        result, err = self.robot_worker.run_auto_calib(vision_idx)

        if err:
            self.append_log(f"[标定] ❌ 异常: {err}")
            self.calib_result_label.setText("标定状态: ❌ 异常")
            self.calib_result_label.setStyleSheet("color: red; font-weight: bold;")
            return

        if result is None:
            self.append_log("[标定] ❌ 失败")
            self.calib_result_label.setText("标定状态: ❌ 失败")
            self.calib_result_label.setStyleSheet("color: red; font-weight: bold;")
            return

        flag = result['flag']
        errors = result['errors']

        if flag == 1:
            avg_x = errors[0]
            avg_y = errors[1]
            max_x = errors[2]
            max_y = errors[3]

            self.append_log("=" * 60)
            self.append_log("[标定] ✅✅✅ 标定成功！")
            self.append_log(f"[标定] X方向平均误差: {avg_x:.4f} mm")
            self.append_log(f"[标定] Y方向平均误差: {avg_y:.4f} mm")
            self.append_log(f"[标定] X方向最大误差: {max_x:.4f} mm")
            self.append_log(f"[标定] Y方向最大误差: {max_y:.4f} mm")

            if avg_x < 1.0 and avg_y < 1.0:
                self.append_log("[标定] ⭐ 精度优秀 (误差 < 1mm)")
                status_text = f"✅ 成功 (X={avg_x:.3f}mm Y={avg_y:.3f}mm)"
                color = "green"
            elif avg_x < 3.0 and avg_y < 3.0:
                self.append_log("[标定] 👍 精度良好 (误差 < 3mm)")
                status_text = f"✅ 良好 (X={avg_x:.3f}mm Y={avg_y:.3f}mm)"
                color = "blue"
            else:
                self.append_log("[标定] ⚠️ 精度一般，建议重新标定")
                status_text = f"⚠️ 一般 (X={avg_x:.3f}mm Y={avg_y:.3f}mm)"
                color = "orange"

            x5v.vision_set_calib_par(self.robot_worker.handle, vision_idx, result['transformation'])
            self.append_log("[标定] 标定参数已自动应用到控制器")

            self.calib_result_label.setText(f"标定状态: {status_text}")
            self.calib_result_label.setStyleSheet(f"color: {color}; font-weight: bold;")

        elif flag == -1:
            self.append_log("[标定] ⚠️ 标定完成但误差超标 (>5mm)")
            self.append_log("[标定] 建议重新标定或检查标定板/视觉识别")
            self.calib_result_label.setText("标定状态: ⚠️ 误差超标")
            self.calib_result_label.setStyleSheet("color: orange; font-weight: bold;")
        else:
            self.append_log("[标定] ❌ 标定失败")
            self.append_log("[标定] 请检查:")
            self.append_log("  1. 标定板是否在相机视野内")
            self.append_log("  2. Vision Master 是否返回 ATTR (0/1/2/3)")
            self.append_log("  3. 数据格式是否为: X,Y,C,ATTR,ID;")
            self.calib_result_label.setText("标定状态: ❌ 失败")
            self.calib_result_label.setStyleSheet("color: red; font-weight: bold;")

        self.append_log("=" * 60)

    # ============================================================
    # 标定验证
    # ============================================================
    def on_verify(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        if not self.latest_vm_points:
            self.append_log("[错误] 请先触发拍照获取视觉数据")
            return

        p = self.latest_vm_points[0]
        vision_idx = int(self.vision_index_combo.currentText().split()[-1])

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
            self.append_log(f"[验证] ❌ 失败: {err}")
            self.verify_result_label.setText(f"验证结果: ❌ {err}")
            return

        self.verify_result_label.setText(
            f"验证结果: 机器人坐标 X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}"
        )
        self.append_log(f"[验证] ✅ 转换后: X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}")

    # ============================================================
    # 日志管理
    # ============================================================
    def append_log(self, msg):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    def clear_log(self):
        """清空日志"""
        self.log_text.clear()
        self.append_log("[日志] 已清空")

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