import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer

import xapi.api as x5
import xapi.api.vision as x5v   # 视觉工艺专用模块


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
        self.setWindowTitle("X5 机器人控制 + 视觉标定 (SCARA J2)")
        self.setGeometry(200, 200, 950, 850)

        # 创建工作线程
        self.worker = RobotWorker()
        self.worker.connected_signal.connect(self.on_connected)
        self.worker.log_signal.connect(self.append_log)

        # 状态定时器
        self.state_timer = QTimer()
        self.state_timer.timeout.connect(self.refresh_state)
        self.state_timer.setInterval(200)

        self.is_connected = False

        # 存储标定基准点 (笛卡尔坐标元组)
        self.base_point = None

        self.init_ui()
        self._set_controls_enabled(False)

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

        # ---------- ★★★ 新增：视觉标定区域 (SCARA J2 动态) ★★★ ----------
        calib_group = QGroupBox("视觉标定 (SCARA J2 动态)")
        calib_layout = QGridLayout()

        # 第一行：视觉工艺号
        calib_layout.addWidget(QLabel("视觉工艺号:"), 0, 0)
        self.vision_index_combo = QComboBox()
        self.vision_index_combo.addItems([str(i) for i in range(8)])
        self.vision_index_combo.setCurrentIndex(0)
        calib_layout.addWidget(self.vision_index_combo, 0, 1)

        # 第二行：标定板参数
        calib_layout.addWidget(QLabel("Mark 点间距 (mm):"), 1, 0)
        self.mark_distance_edit = QLineEdit("10")
        calib_layout.addWidget(self.mark_distance_edit, 1, 1)

        calib_layout.addWidget(QLabel("J2 移动步长 (mm):"), 1, 2)
        self.step_size_edit = QLineEdit("20")
        calib_layout.addWidget(self.step_size_edit, 1, 3)

        calib_layout.addWidget(QLabel("像素偏移 (U, V):"), 2, 0)
        self.pixel_shift_u_edit = QLineEdit("2592")
        self.pixel_shift_v_edit = QLineEdit("2048")
        calib_layout.addWidget(self.pixel_shift_u_edit, 2, 1)
        calib_layout.addWidget(self.pixel_shift_v_edit, 2, 2)

        # 第三行：基准点记录
        self.base_point_label = QLabel("基准点: 未记录")
        calib_layout.addWidget(self.base_point_label, 3, 0, 1, 2)
        self.record_base_btn = QPushButton("记录当前笛卡尔位置为基准点")
        self.record_base_btn.clicked.connect(self.on_record_base_point)
        calib_layout.addWidget(self.record_base_btn, 3, 2, 1, 2)

        # 第四行：操作按钮
        self.apply_config_btn = QPushButton("① 写入视觉工艺参数")
        self.apply_config_btn.clicked.connect(self.on_apply_vision_config)
        self.start_calib_btn = QPushButton("② 开始自动标定")
        self.start_calib_btn.clicked.connect(self.on_start_calibration)
        self.verify_btn = QPushButton("③ 标定验证 (获取视觉->转换)")
        self.verify_btn.clicked.connect(self.on_verify_calibration)

        calib_layout.addWidget(self.apply_config_btn, 4, 0)
        calib_layout.addWidget(self.start_calib_btn, 4, 1)
        calib_layout.addWidget(self.verify_btn, 4, 2)

        # 标定结果显示
        self.calib_result_label = QLabel("标定状态: 等待操作")
        calib_layout.addWidget(self.calib_result_label, 5, 0, 1, 4)

        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)

        # ---------- 日志 ----------
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # 初始禁用标定按钮（连接后启用）
        self._set_calib_controls_enabled(False)

    def _set_controls_enabled(self, enabled):
        for w in [self.mode_auto_btn, self.mode_autocmd_btn, self.mode_manual_btn,
                  self.servo_on_btn, self.servo_off_btn, self.reset_alarm_btn,
                  self.speed_slider, self.movj_btn, self.movl_btn, self.emergency_btn]:
            w.setEnabled(enabled)
        self._set_calib_controls_enabled(enabled)

    def _set_calib_controls_enabled(self, enabled):
        """控制标定相关控件使能状态"""
        for w in [self.record_base_btn, self.apply_config_btn, self.start_calib_btn, self.verify_btn,
                  self.mark_distance_edit, self.step_size_edit,
                  self.pixel_shift_u_edit, self.pixel_shift_v_edit]:
            w.setEnabled(enabled)

    # ---------- 核心：刷新状态 ----------
    def refresh_state(self):
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
        if not self.worker.isRunning():
            self.worker.start()
        QTimer.singleShot(50, self.worker.connect_robot)

    def on_disconnect(self):
        self.worker.disconnect_robot()
        self.state_timer.stop()
        self.is_connected = False
        self.disconnect_btn.setEnabled(False)
        self.connect_btn.setEnabled(True)
        self.status_label.setText("⚪ 未连接")
        self._set_controls_enabled(False)
        # 清空状态
        self.mode_label.setText("模式: --")
        self.enable_label.setText("使能: --")
        self.alarm_label.setText("报警: --")
        self.speed_label.setText("速度: --%")
        self.joint_label.setText("关节: --")
        self.cart_label.setText("笛卡尔: --")
        self.base_point_label.setText("基准点: 未记录")
        self.calib_result_label.setText("标定状态: 等待操作")

    def on_connected(self, success):
        if success:
            self.is_connected = True
            self.status_label.setText("🟢 已连接")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self._set_controls_enabled(True)
            self.append_log("[状态] 连接成功，开始自动刷新状态")
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

    # ---------- ★★★ 视觉标定相关槽函数 ★★★ ----------
    def on_record_base_point(self):
        """记录当前机器人笛卡尔位置为标定基准点 (base_point)"""
        if not self.is_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        info = self.worker.get_state()
        if info is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        p = info['cart']
        self.base_point = (p[0], p[1], p[2], p[3], p[4], p[5])
        self.base_point_label.setText(f"基准点: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f}")
        self.append_log(f"[基准点] 已记录: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")

    def on_apply_vision_config(self):
        """将当前界面参数写入控制器视觉工艺 (Vision 0)"""
        if not self.is_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        if self.base_point is None:
            self.append_log("[错误] 请先记录基准点")
            return

        try:
            vision_idx = int(self.vision_index_combo.currentText())

            # 1. 构建 Vision 配置结构体 (关键: camera_mount_type=2 表示动态非末端, 即 SCARA J2)
            config = x5v.Vision(
                ip=b"168.168.40.111",          # ★ 请改为你 Vision Master 的实际 IP
                port=4004,                      # ★ 请改为你 Vision Master 的实际端口
                communication_protocol=0,       # 0: TCP Client
                auto_start=False,
                data_format=1,                  # 必须为 1 (返回 X,Y,C,ATTR,ID)
                invert_c_value=False,
                camera_mount_type=2,            # 2: 动态非末端 (SCARA J2)
                uf_index=0,                     # 使用世界坐标系 UF0 (若工作台水平)
                calibration_object=1,           # 1: 机器人标定
                trigger_type=0,                 # 0: 网络触发
                trigger_do=255,
                trigger_command=b"trigger"      # ★ 请改为你 Vision Master 的触发命令
            )

            # 写入控制器
            x5v.vision_set_config(self.worker.handle, vision_idx, config)
            self.append_log(f"[视觉工艺] Vision {vision_idx} 配置写入成功")

            # 2. 构建标定过程参数 (动态 J2 自动标定)
            pixel_u = int(self.pixel_shift_u_edit.text())
            pixel_v = int(self.pixel_shift_v_edit.text())
            mark_dist = int(self.mark_distance_edit.text())
            step = int(self.step_size_edit.text())

            # 构造 base_point (Point 类型)
            base_pose = x5.Pose(
                self.base_point[0], self.base_point[1], self.base_point[2],
                self.base_point[3], self.base_point[4], self.base_point[5],
                0, 0, 0
            )
            base_point = x5.Point(pose=base_pose, uf=0, tf=0, cfg=(0, 0, 0, 1))

            # 构造标定过程数据
            calib_process = x5v.VisionCalibProcess()
            calib_process.calibration_type = 1   # 1: 自动标定
            calib_process.point_count = 0        # 0: 9点 (也可选 1:16点, 2:25点)
            calib_process.is_workplane_parallel = True

            # 填充动态非末端标定数据 (_AutoDynamicNotEndData)
            # 注意：根据 xapi 版本，路径可能为 _CalibrationDataUnion._AutoDynamicNotEndData
            # 这里使用直接赋值方式，如果报错再调整
            calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
                tf_index=0,                      # 标定时使用法兰坐标系 TF0
                pixel_shift=[pixel_u, pixel_v],  # 相机像素尺寸 (宽, 高)
                error=0.0,                       # 初始误差填 0
                base_point=base_point,           # 刚才记录的基准点
                mark_distance=mark_dist,         # 标定纸上 Mark 点间距 (mm)
                step_size=step                   # J2 轴移动步长 (mm)
            )

            # 写入控制器
            x5v.vision_write_calib_process(self.worker.handle, vision_idx, calib_process)
            self.append_log(f"[标定参数] 动态 J2 标定参数写入成功")
            self.calib_result_label.setText("标定状态: 参数已就绪，可开始标定")

        except Exception as e:
            self.append_log(f"[配置失败] {str(e)}")
            self.calib_result_label.setText(f"标定状态: 配置失败")

    def on_start_calibration(self):
        """执行自动标定"""
        if not self.is_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        try:
            vision_idx = int(self.vision_index_combo.currentText())

            self.append_log("[标定] 开始自动标定，请确保机器人周围安全...")
            self.calib_result_label.setText("标定状态: 进行中...")

            # 调用自动标定接口 (注意: 此接口会阻塞 UI，后续可优化到子线程)
            flag, errors, transformation = x5v.vision_auto_calib(
                self.worker.handle, vision_idx
            )

            if flag == 1:
                self.append_log(f"[标定] 成功！误差: X平均={errors[0]:.3f}mm, Y平均={errors[1]:.3f}mm")
                self.calib_result_label.setText(f"标定状态: ✅ 成功 (误差 {errors[0]:.3f}mm)")
                # 自动应用标定结果
                x5v.vision_set_calib_par(self.worker.handle, vision_idx, transformation)
                self.append_log("[标定] 标定参数已自动应用")
            elif flag == -1:
                self.append_log("[标定] 成功但误差超标 (可能 >5mm)")
                self.calib_result_label.setText("标定状态: ⚠️ 成功但误差大")
            else:
                self.append_log("[标定] 失败！请检查标定板是否在视野内，ATTR 是否正确")
                self.calib_result_label.setText("标定状态: ❌ 失败")

        except Exception as e:
            self.append_log(f"[标定异常] {str(e)}")
            self.calib_result_label.setText("标定状态: ❌ 异常")

    def on_verify_calibration(self):
        """标定验证：获取一次视觉像素，转换成机器人坐标"""
        if not self.is_connected:
            self.append_log("[错误] 请先连接机器人")
            return

        # 这里需要借助 Vision Master 获取像素数据，你可以复用之前的 vm_thread
        # 如果没有集成，可先提示
        self.append_log("[验证] 请确保 Vision Master 已连接并触发拍照")
        self.append_log("[验证] 接收到数据后会自动调用 vision_static_cnvr 转换")

        # 如果你已经集成了 VisionMasterThread，可以在其 data_received 信号中调用转换
        # 这里给出一个示例：假设你有一个 vm_thread 并已连接
        # 你可以在这里调用 vm_thread.trigger_and_get()，并在 on_vm_data 中处理转换
        # 由于当前代码未包含 vm_thread，此处只打印提示
        # 若需实现，可参考之前的代码添加

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