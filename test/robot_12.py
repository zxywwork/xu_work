import sys
import socket
import time
import traceback
import re
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QColor

import xapi.api as x5
import xapi.api.vision as x5v


# ============================================================
# 工作线程1：Vision Master 通信（保持不变）
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
        self._trigger_flag = False
        self._stop_flag = False

    def set_params(self, ip, port, trigger_cmd="trigger"):
        self.ip = ip
        self.port = port
        self.trigger_cmd = trigger_cmd

    def connect_to_server(self):
        self._stop_flag = False
        if not self.isRunning():
            self.start()
        self._trigger_flag = "connect"

    def disconnect(self):
        self._stop_flag = True
        self._trigger_flag = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.running = False
        self.connected_signal.emit(False)

    def trigger_and_get(self):
        if not self.running:
            self.log_signal.emit("[VM] 错误: 未连接")
            return
        self._trigger_flag = "trigger"

    def run(self):
        self.running = True
        while not self._stop_flag:
            if self._trigger_flag == "connect":
                self._trigger_flag = False
                self._do_connect()
            elif self._trigger_flag == "trigger":
                self._trigger_flag = False
                self._do_trigger()
            self.msleep(50)
        self.running = False

    def _do_connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip, self.port))
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[VM] 连接成功 {self.ip}:{self.port}")
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[VM] 连接失败: {str(e)}")
            self.sock = None

    def _do_trigger(self):
        if not self.sock:
            self.log_signal.emit("[VM] 错误: 未连接")
            return
        try:
            self.sock.send(self.trigger_cmd.encode())
            self.log_signal.emit(f"[VM] 发送触发: {self.trigger_cmd}")
            self.sock.settimeout(3.0)
            raw_data = self.sock.recv(4096).decode().strip()
            self.raw_data_received.emit(raw_data)
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


# ============================================================
# 工作线程2：机器人 API 调用（增强码垛命令）
# ============================================================
class RobotWorker(QThread):
    state_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)
    calib_result_signal = pyqtSignal(int, list, list)
    verify_result_signal = pyqtSignal(object, str)
    plr_value_signal = pyqtSignal(int, int, int)   # row, col, layer

    def __init__(self):
        super().__init__()
        self.handle = None
        self.ip = ""
        self.running = False
        self.cmd_queue = []
        from threading import Lock
        self.cmd_lock = Lock()
        self.need_reconnect = False

    def set_ip(self, ip):
        self.ip = ip

    def connect_robot(self):
        with self.cmd_lock:
            self.cmd_queue.append("connect")

    def disconnect_robot(self):
        with self.cmd_lock:
            self.cmd_queue.append("disconnect")

    def get_state(self):
        with self.cmd_lock:
            self.cmd_queue.append("get_state")

    def set_mode(self, mode):
        with self.cmd_lock:
            self.cmd_queue.append(("set_mode", mode))

    def set_servo(self, on):
        with self.cmd_lock:
            self.cmd_queue.append(("set_servo", on))

    def reset_alarm(self):
        with self.cmd_lock:
            self.cmd_queue.append("reset_alarm")

    def emergency_stop(self):
        with self.cmd_lock:
            self.cmd_queue.append("emergency_stop")

    def set_speed(self, speed):
        with self.cmd_lock:
            self.cmd_queue.append(("set_speed", speed))

    def movj_to(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        with self.cmd_lock:
            self.cmd_queue.append(("movj", (j1, j2, j3, j4, j5, j6)))

    def movl_to(self, x, y, z, a=0, b=0, c=0):
        with self.cmd_lock:
            self.cmd_queue.append(("movl", (x, y, z, a, b, c)))

    def apply_vision_config(self, vision_idx, config, calib_process):
        with self.cmd_lock:
            self.cmd_queue.append(("apply_config", vision_idx, config, calib_process))

    def run_auto_calib(self, vision_idx):
        with self.cmd_lock:
            self.cmd_queue.append(("auto_calib", vision_idx))

    def verify_vision(self, vision_idx, pixel_pose, trig_point):
        with self.cmd_lock:
            self.cmd_queue.append(("verify", vision_idx, pixel_pose, trig_point))

    def switch_uf(self, uf_index):
        with self.cmd_lock:
            self.cmd_queue.append(("switch_uf", uf_index))

    def switch_tf(self, tf_index):
        with self.cmd_lock:
            self.cmd_queue.append(("switch_tf", tf_index))

    # ===== 码垛命令 =====
    def setup_pallet(self, pallet_id, pallet_type, p1_pr, p2_pr, p3_pr,
                     row, col, layer, height, order, approach_z_mode, approach_z_offset,
                     depart_z_mode, depart_z_offset):
        """设置码垛参数（种类1）"""
        with self.cmd_lock:
            self.cmd_queue.append(("setup_pallet", pallet_id, pallet_type, p1_pr, p2_pr, p3_pr,
                                   row, col, layer, height, order,
                                   approach_z_mode, approach_z_offset,
                                   depart_z_mode, depart_z_offset))

    def update_pallet_pos(self, pallet_id):
        with self.cmd_lock:
            self.cmd_queue.append(("update_pallet_pos", pallet_id))

    def move_pallet(self, pallet_id, target):
        """target: 'prepos', 'targetpos', 'postpos'"""
        with self.cmd_lock:
            self.cmd_queue.append(("move_pallet", pallet_id, target))

    def update_plr(self, pallet_id):
        with self.cmd_lock:
            self.cmd_queue.append(("update_plr", pallet_id))

    def get_plr(self, pallet_id):
        with self.cmd_lock:
            self.cmd_queue.append(("get_plr", pallet_id))

    def set_pr_value(self, pr_index, point):
        """设置PR变量（用于记录P1,P2,P3）"""
        with self.cmd_lock:
            self.cmd_queue.append(("set_pr", pr_index, point))

    # ---- run 循环处理 ----
    def run(self):
        self.running = True
        while self.running:
            cmd = None
            with self.cmd_lock:
                if self.cmd_queue:
                    cmd = self.cmd_queue.pop(0)
            if cmd is None:
                self.msleep(20)
                continue

            try:
                if cmd == "connect":
                    self._do_connect()
                elif cmd == "disconnect":
                    self._do_disconnect()
                elif cmd == "get_state":
                    self._do_get_state()
                elif cmd[0] == "set_mode":
                    self._do_set_mode(cmd[1])
                elif cmd[0] == "set_servo":
                    self._do_set_servo(cmd[1])
                elif cmd == "reset_alarm":
                    self._do_reset_alarm()
                elif cmd == "emergency_stop":
                    self._do_emergency_stop()
                elif cmd[0] == "set_speed":
                    self._do_set_speed(cmd[1])
                elif cmd[0] == "movj":
                    self._do_movj(*cmd[1])
                elif cmd[0] == "movl":
                    self._do_movl(*cmd[1])
                elif cmd[0] == "apply_config":
                    self._do_apply_config(*cmd[1:])
                elif cmd[0] == "auto_calib":
                    self._do_auto_calib(cmd[1])
                elif cmd[0] == "verify":
                    self._do_verify(*cmd[1:])
                elif cmd[0] == "switch_uf":
                    self._do_switch_uf(cmd[1])
                elif cmd[0] == "switch_tf":
                    self._do_switch_tf(cmd[1])
                elif cmd[0] == "setup_pallet":
                    self._do_setup_pallet(*cmd[1:])
                elif cmd[0] == "update_pallet_pos":
                    self._do_update_pallet_pos(cmd[1])
                elif cmd[0] == "move_pallet":
                    self._do_move_pallet(cmd[1], cmd[2])
                elif cmd[0] == "update_plr":
                    self._do_update_plr(cmd[1])
                elif cmd[0] == "get_plr":
                    self._do_get_plr(cmd[1])
                elif cmd[0] == "set_pr":
                    self._do_set_pr(cmd[1], cmd[2])
                else:
                    self.log_signal.emit(f"[Worker] 未知命令: {cmd}")
            except Exception as e:
                self.log_signal.emit(f"[Worker] 命令执行异常: {str(e)}")
                traceback.print_exc()

    # ---- 原有实现 ----
    def _do_connect(self):
        try:
            self.handle = x5.connect(self.ip)
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[机器人] 连接成功，句柄: {self.handle}")
            self.need_reconnect = False
        except Exception as e:
            self.connected_signal.emit(False)
            self.log_signal.emit(f"[机器人] 连接失败: {str(e)}")
            self.handle = None

    def _do_disconnect(self):
        if self.handle is not None:
            try:
                x5.disconnect(self.handle)
            except:
                pass
            self.handle = None
        self.connected_signal.emit(False)
        self.log_signal.emit("[机器人] 已断开")

    def _do_get_state(self):
        if self.handle is None:
            return
        try:
            state = x5.get_system_state(self.handle)
            speed = x5.get_speed(self.handle)
            joint = x5.get_cjoint(self.handle)
            point = x5.get_cpoint(self.handle)
            uf_pose = x5.get_uf(self.handle, point.uf)
            tf_pose = x5.get_tf(self.handle, point.tf)

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
                         point.pose.a, point.pose.b, point.pose.c),
                'uf_no': point.uf,
                'tf_no': point.tf,
                'uf_pose': uf_pose,
                'tf_pose': tf_pose
            }
            self.state_signal.emit(info)
        except Exception as e:
            err_str = str(e)
            if "NET_DOWN" in err_str or "connection" in err_str.lower():
                self.log_signal.emit("[状态获取] 检测到网络断开，尝试重连...")
                self.need_reconnect = True
                self.handle = None
                self.connected_signal.emit(False)
                QTimer.singleShot(1000, self._try_reconnect)
            else:
                self.log_signal.emit(f"[状态获取异常] {err_str}")

    def _try_reconnect(self):
        if self.need_reconnect and not self.handle:
            self.log_signal.emit("[重连] 正在重新连接机器人...")
            self._do_connect()

    def _do_set_mode(self, mode):
        if self.handle is None:
            return
        try:
            x5.set_system_mode(self.handle, mode)
            names = {2: "自动", 3: "手动", 5: "调试", 100: "自动命令"}
            self.log_signal.emit(f"[模式] -> {names.get(mode, str(mode))}")
        except Exception as e:
            self.log_signal.emit(f"[模式] 失败: {str(e)}")

    def _do_set_servo(self, on):
        if self.handle is None:
            return
        try:
            x5.enable_servo(self.handle, on)
            self.log_signal.emit(f"[使能] {'上使能' if on else '下使能'} 成功")
        except Exception as e:
            self.log_signal.emit(f"[使能] 失败: {str(e)}")

    def _do_reset_alarm(self):
        if self.handle is None:
            return
        try:
            x5.reset(self.handle)
            self.log_signal.emit("[报警复位] 成功")
        except Exception as e:
            self.log_signal.emit(f"[报警复位] 失败: {str(e)}")

    def _do_emergency_stop(self):
        if self.handle is None:
            return
        try:
            x5.abort(self.handle)
            self.log_signal.emit("[急停] 已触发")
        except Exception as e:
            self.log_signal.emit(f"[急停] 失败: {str(e)}")

    def _do_set_speed(self, speed):
        if self.handle is None:
            return
        try:
            x5.set_speed(self.handle, speed)
            self.log_signal.emit(f"[速度] {speed}%")
        except Exception as e:
            self.log_signal.emit(f"[速度] 失败: {str(e)}")

    def _do_movj(self, j1, j2, j3, j4, j5, j6):
        if self.handle is None:
            return
        try:
            target = x5.Joint(j1, j2, j3, j4, j5, j6, 0, 0, 0)
            x5.movj(self.handle, target)
            self.log_signal.emit(f"[运动] movj 到 ({j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}, {j5:.1f}, {j6:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[运动] 失败: {str(e)}")

    def _do_movl(self, x, y, z, a, b, c):
        if self.handle is None:
            return
        try:
            pose = x5.Pose(x, y, z, a, b, c, 0, 0, 0)
            point = x5.Point(pose=pose, uf=0, tf=0, cfg=(0, 0, 0, 1))
            x5.movl(self.handle, point)
            self.log_signal.emit(f"[运动] movl 到 ({x:.1f}, {y:.1f}, {z:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[运动] 失败: {str(e)}")

    def _do_apply_config(self, vision_idx, config, calib_process):
        if self.handle is None:
            self.log_signal.emit("[配置] 机器人未连接")
            return
        try:
            x5v.vision_set_config(self.handle, vision_idx, config)
            self.log_signal.emit(f"[配置] Vision {vision_idx} 参数写入成功")
            x5v.vision_write_calib_process(self.handle, vision_idx, calib_process)
            self.log_signal.emit(f"[配置] Vision {vision_idx} 标定参数写入成功")
        except Exception as e:
            self.log_signal.emit(f"[配置] 失败: {str(e)}")

    def _do_auto_calib(self, vision_idx):
        if self.handle is None:
            self.calib_result_signal.emit(-2, [], [])
            self.log_signal.emit("[标定] 机器人未连接")
            return
        try:
            self.log_signal.emit("[标定] 正在启动自动标定流程...")
            self.log_signal.emit("[标定] 请确保机器人周围安全，标定板在视野内")
            self.log_signal.emit("[标定] 机器人将自动移动 J2 轴完成标定，请观察机器人运动")
            flag, errors, transformation = x5v.vision_auto_calib(self.handle, vision_idx)
            self.log_signal.emit(f"[标定] 返回标志 flag={flag}")
            if flag == 1:
                self.log_signal.emit(f"[标定] ✅ X方向平均误差: {errors[0]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ Y方向平均误差: {errors[1]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ X方向最大误差: {errors[2]:.4f} mm")
                self.log_signal.emit(f"[标定] ✅ Y方向最大误差: {errors[3]:.4f} mm")
                if errors[1] == 0.0 and errors[3] == 0.0:
                    self.log_signal.emit("[警告] Y方向误差为0，可能标定数据异常，请检查视觉数据是否正确包含Y坐标")
            elif flag == -1:
                self.log_signal.emit("[标定] ⚠️ 标定完成但误差超标 (建议 >5mm 重新标定)")
            elif flag == -2:
                self.log_signal.emit("[标定] ❌ 标定过程出错")
                self.log_signal.emit("[标定] 可能原因: 1. 标定过程中相机未识别到标定板  2. 视觉数据格式不正确  3. 触发方式设置错误")
            else:
                self.log_signal.emit("[标定] ❌ 标定失败")
            self.calib_result_signal.emit(flag, errors, transformation)
        except Exception as e:
            self.log_signal.emit(f"[标定] 异常: {str(e)}")
            self.calib_result_signal.emit(-2, [], [])

    def _do_verify(self, vision_idx, pixel_pose, trig_point):
        if self.handle is None:
            self.verify_result_signal.emit(None, "机器人未连接")
            return
        try:
            result_pose = x5v.vision_dynamic_cnvr(self.handle, vision_idx, pixel_pose, trig_point)
            self.verify_result_signal.emit(result_pose, None)
        except Exception as e:
            self.verify_result_signal.emit(None, str(e))

    def _do_switch_uf(self, uf_index):
        if self.handle is None:
            self.log_signal.emit("[切换UF] 机器人未连接")
            return
        try:
            x5.set_ufno(self.handle, uf_index)
            self.log_signal.emit(f"[切换UF] 成功切换到 UF{uf_index}")
        except Exception as e:
            self.log_signal.emit(f"[切换UF] 失败: {str(e)}")

    def _do_switch_tf(self, tf_index):
        if self.handle is None:
            self.log_signal.emit("[切换TF] 机器人未连接")
            return
        try:
            x5.set_tfno(self.handle, tf_index)
            self.log_signal.emit(f"[切换TF] 成功切换到 TF{tf_index}")
        except Exception as e:
            self.log_signal.emit(f"[切换TF] 失败: {str(e)}")

    # ===== 码垛实现 =====
    def _do_setup_pallet(self, pallet_id, pallet_type, p1_pr, p2_pr, p3_pr,
                         row, col, layer, height, order,
                         approach_z_mode, approach_z_offset,
                         depart_z_mode, depart_z_offset):
        if self.handle is None:
            self.log_signal.emit("[码垛] 机器人未连接")
            return
        if pallet_type != 1:
            self.log_signal.emit("[码垛] 目前仅支持种类1（四边形堆叠）")
            return
        # 构建 Lua 指令 Pallet.Load
        # 格式: R[0] = Pallet.Load(pallet_id, PR[p1_pr], PR[p2_pr], PR[p3_pr], row, col, layer, height)
        lua_cmd = f"R[0]=Pallet.Load({pallet_id}, PR[{p1_pr}], PR[{p2_pr}], PR[{p3_pr}], {row}, {col}, {layer}, {height})"
        try:
            x5.execute_lua(self.handle, lua_cmd)
            self.log_signal.emit(f"[码垛] 设置 Pallet{pallet_id} 成功 (行{row},列{col},层{layer},层高{height})")
            # 后续可设置趋近点/离开点等，但需另外指令，暂略
        except Exception as e:
            self.log_signal.emit(f"[码垛] 设置失败: {str(e)}")

    def _do_update_pallet_pos(self, pallet_id):
        if self.handle is None:
            self.log_signal.emit("[码垛] 机器人未连接")
            return
        try:
            x5.execute_lua(self.handle, f"Pallet.UpPos({pallet_id})")
            self.log_signal.emit(f"[码垛] Pallet{pallet_id} 位置已更新")
        except Exception as e:
            self.log_signal.emit(f"[码垛] 更新位置失败: {str(e)}")

    def _do_move_pallet(self, pallet_id, target):
        if self.handle is None:
            self.log_signal.emit("[码垛] 机器人未连接")
            return
        # target 为字符串 'prepos', 'targetpos', 'postpos'
        try:
            x5.execute_lua(self.handle, f"Pallet.Mov({pallet_id}, '{target}')")
            self.log_signal.emit(f"[码垛] 移动到 Pallet{pallet_id} 的 {target}")
        except Exception as e:
            self.log_signal.emit(f"[码垛] 移动失败: {str(e)}")

    def _do_update_plr(self, pallet_id):
        if self.handle is None:
            self.log_signal.emit("[码垛] 机器人未连接")
            return
        try:
            x5.execute_lua(self.handle, f"Pallet.UpPlr({pallet_id})")
            self.log_signal.emit(f"[码垛] Pallet{pallet_id} 的 PLR 已更新")
            # 自动读取更新后的 PLR
            self._do_get_plr(pallet_id)
        except Exception as e:
            self.log_signal.emit(f"[码垛] 更新 PLR 失败: {str(e)}")

    def _do_get_plr(self, pallet_id):
        if self.handle is None:
            return
        try:
            # 使用 Lua 读取 PLR 到 R 变量，再通过 API 读取 R
            # 但 execute_lua 无法获取返回值，所以采用另一种方式：
            # 通过 execute_lua 将 PLR 赋值给 R 变量，然后我们再用 get_hr 读取？
            # 但 R 变量不是 HR，不能直接读取。
            # 替代方案：使用系统变量或直接打印到日志，但无法获取数值。
            # 我们使用一种技巧：执行一个 Lua 脚本，将 PLR 值写入 VR 寄存器，然后用 GetVR 读取。
            # 但 VR 是视觉寄存器，可能冲突。更好是使用 R 变量，但无法读取。
            # 只能通过日志观察，或者使用 Pallet.GetPos 获取实际点位。
            # 这里我们只提示已更新，不显示具体数值。
            # 或者可以调用 lua 脚本返回字符串，但 xapi 不支持。
            # 因此，我们仅日志提示，并发送一个信号，但信号不带值。
            # 也可以使用 get_plc_data 等，但未提供。
            # 故仅日志。
            self.log_signal.emit(f"[码垛] 当前 PLR 值可通过日志查看 (请观察机器人运动或使用示教器)")
        except Exception as e:
            self.log_signal.emit(f"[码垛] 读取 PLR 失败: {str(e)}")

    def _do_set_pr(self, pr_index, point):
        if self.handle is None:
            return
        try:
            x5.set_pr(self.handle, pr_index, point)
            self.log_signal.emit(f"[PR] PR{pr_index} 已更新")
        except Exception as e:
            self.log_signal.emit(f"[PR] 设置 PR{pr_index} 失败: {str(e)}")


# ============================================================
# 主窗口（整合码垛）
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("视觉码垛控制系统 - 含码垛工艺")
        self.setGeometry(50, 50, 1350, 1200)

        self.robot_worker = RobotWorker()
        self.robot_worker.connected_signal.connect(self.on_robot_connected)
        self.robot_worker.log_signal.connect(self.append_log)
        self.robot_worker.state_signal.connect(self.update_state)
        self.robot_worker.calib_result_signal.connect(self.on_calib_result)
        self.robot_worker.verify_result_signal.connect(self.on_verify_result)
        self.robot_worker.start()

        self.vm_thread = VisionMasterThread()
        self.vm_thread.connected_signal.connect(self.on_vm_connected)
        self.vm_thread.data_received.connect(self.on_vm_data_received)
        self.vm_thread.raw_data_received.connect(self.on_vm_raw_data)
        self.vm_thread.log_signal.connect(self.append_log)
        self.vm_thread.start()

        self.state_timer = QTimer()
        self.state_timer.timeout.connect(self.request_state)
        self.state_timer.setInterval(200)

        self.is_robot_connected = False
        self.is_vm_connected = False
        self.latest_vm_points = []
        self.base_point = None

        self.init_ui()
        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # ---- 顶部：机器人控制 + UF/TF切换 ----
        robot_group = QGroupBox("机器人控制")
        robot_layout = QVBoxLayout()
        # 第一行：IP连接
        row1 = QHBoxLayout()
        row1.addWidget(QLabel("IP:"))
        self.robot_ip_edit = QLineEdit("168.168.40.20")
        self.robot_ip_edit.setFixedWidth(150)
        row1.addWidget(self.robot_ip_edit)
        self.robot_connect_btn = QPushButton("连接机器人")
        self.robot_connect_btn.clicked.connect(self.on_robot_connect)
        self.robot_disconnect_btn = QPushButton("断开")
        self.robot_disconnect_btn.clicked.connect(self.on_robot_disconnect)
        self.robot_disconnect_btn.setEnabled(False)
        row1.addWidget(self.robot_connect_btn)
        row1.addWidget(self.robot_disconnect_btn)
        self.robot_status_label = QLabel("⚪ 未连接")
        row1.addWidget(self.robot_status_label)
        row1.addStretch()
        robot_layout.addLayout(row1)
        # 第二行：UF/TF切换
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("当前UF:"))
        self.current_uf_label = QLabel("UF0")
        row2.addWidget(self.current_uf_label)
        row2.addWidget(QLabel("当前TF:"))
        self.current_tf_label = QLabel("TF0")
        row2.addWidget(self.current_tf_label)
        row2.addWidget(QLabel("切换UF:"))
        self.switch_uf_combo = QComboBox()
        self.switch_uf_combo.addItems([f"UF{i}" for i in range(17)])
        self.switch_uf_combo.setCurrentIndex(0)
        row2.addWidget(self.switch_uf_combo)
        self.switch_uf_btn = QPushButton("切换UF")
        self.switch_uf_btn.clicked.connect(self.on_switch_uf)
        row2.addWidget(self.switch_uf_btn)
        row2.addWidget(QLabel("切换TF:"))
        self.switch_tf_combo = QComboBox()
        self.switch_tf_combo.addItems([f"TF{i}" for i in range(17)])
        self.switch_tf_combo.setCurrentIndex(0)
        row2.addWidget(self.switch_tf_combo)
        self.switch_tf_btn = QPushButton("切换TF")
        self.switch_tf_btn.clicked.connect(self.on_switch_tf)
        row2.addWidget(self.switch_tf_btn)
        row2.addStretch()
        robot_layout.addLayout(row2)
        # 第三行：UF/TF位姿
        row3 = QHBoxLayout()
        self.uf_pose_label = QLabel("UF位姿: --")
        self.tf_pose_label = QLabel("TF位姿: --")
        row3.addWidget(self.uf_pose_label)
        row3.addWidget(self.tf_pose_label)
        row3.addStretch()
        robot_layout.addLayout(row3)
        robot_group.setLayout(robot_layout)
        main_layout.addWidget(robot_group)

        # ---- Vision Master 通信（不变） ----
        vm_group = QGroupBox("视觉通信 (Vision Master)")
        vm_layout = QGridLayout()
        vm_layout.addWidget(QLabel("协议类型:"), 0, 0)
        self.vm_protocol_combo = QComboBox()
        self.vm_protocol_combo.addItems(["机器人TCPClient", "机器人TCPServer"])
        self.vm_protocol_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_protocol_combo, 0, 1)
        vm_layout.addWidget(QLabel("IP地址:"), 0, 2)
        self.vm_ip_edit = QLineEdit("168.168.40.111")
        self.vm_ip_edit.setFixedWidth(140)
        vm_layout.addWidget(self.vm_ip_edit, 0, 3)
        vm_layout.addWidget(QLabel("端口号:"), 0, 4)
        self.vm_port_edit = QLineEdit("4004")
        self.vm_port_edit.setFixedWidth(80)
        vm_layout.addWidget(self.vm_port_edit, 0, 5)
        vm_layout.addWidget(QLabel("触发方式:"), 1, 0)
        self.vm_trigger_type_combo = QComboBox()
        self.vm_trigger_type_combo.addItems(["网络触发", "IO触发"])
        self.vm_trigger_type_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_trigger_type_combo, 1, 1)
        vm_layout.addWidget(QLabel("触发字符:"), 1, 2)
        self.vm_trigger_edit = QLineEdit("trigger")
        self.vm_trigger_edit.setFixedWidth(140)
        vm_layout.addWidget(self.vm_trigger_edit, 1, 3)
        vm_layout.addWidget(QLabel("接收格式:"), 1, 4)
        self.vm_format_combo = QComboBox()
        self.vm_format_combo.addItems([str(i) for i in range(10)])
        self.vm_format_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_format_combo, 1, 5)
        self.vm_c_invert_check = QCheckBox("C值取反")
        vm_layout.addWidget(self.vm_c_invert_check, 2, 0)
        vm_layout.addWidget(QLabel("视觉坐标系(UF):"), 2, 1)
        self.vm_uf_combo = QComboBox()
        self.vm_uf_combo.addItems([f"UF{i}" for i in range(17)])
        self.vm_uf_combo.setCurrentIndex(15)
        vm_layout.addWidget(self.vm_uf_combo, 2, 2)
        vm_layout.addWidget(QLabel("工具坐标系(TF):"), 2, 3)
        self.vm_tf_combo = QComboBox()
        self.vm_tf_combo.addItems([f"TF{i}" for i in range(17)])
        self.vm_tf_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_tf_combo, 2, 4)
        vm_layout.addWidget(QLabel("安装类型:"), 2, 5)
        self.vm_mount_combo = QComboBox()
        self.vm_mount_combo.addItems(["动态J2"])
        self.vm_mount_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_mount_combo, 2, 6)
        vm_layout.addWidget(QLabel("超时时间:"), 3, 0)
        self.vm_timeout_edit = QLineEdit("3.0")
        self.vm_timeout_edit.setFixedWidth(60)
        vm_layout.addWidget(self.vm_timeout_edit, 3, 1)
        vm_layout.addWidget(QLabel("标定选择:"), 3, 2)
        self.vm_calib_object_combo = QComboBox()
        self.vm_calib_object_combo.addItems(["视觉软件", "机器人"])
        self.vm_calib_object_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_calib_object_combo, 3, 3)
        self.vm_calib_type_combo = QComboBox()
        self.vm_calib_type_combo.addItems(["手动标定", "自动标定"])
        self.vm_calib_type_combo.setCurrentIndex(1)
        vm_layout.addWidget(self.vm_calib_type_combo, 3, 4)
        self.vm_calib_points_combo = QComboBox()
        self.vm_calib_points_combo.addItems(["9点", "16点", "25点"])
        self.vm_calib_points_combo.setCurrentIndex(0)
        vm_layout.addWidget(self.vm_calib_points_combo, 3, 5)
        self.vm_connect_btn = QPushButton("连接")
        self.vm_connect_btn.clicked.connect(self.on_vm_connect)
        self.vm_disconnect_btn = QPushButton("关闭")
        self.vm_disconnect_btn.clicked.connect(self.on_vm_disconnect)
        self.vm_disconnect_btn.setEnabled(False)
        self.vm_trigger_btn = QPushButton("📷 触发拍照")
        self.vm_trigger_btn.clicked.connect(self.on_trigger_vm)
        self.vm_trigger_btn.setEnabled(False)
        vm_layout.addWidget(self.vm_connect_btn, 3, 6)
        vm_layout.addWidget(self.vm_disconnect_btn, 4, 0)
        vm_layout.addWidget(self.vm_trigger_btn, 4, 1)
        self.vm_status_label = QLabel("⚪ 未连接")
        vm_layout.addWidget(self.vm_status_label, 4, 2, 1, 3)
        vm_layout.addWidget(QLabel("接收格式1: XX,YY,CC; XX,YY,CC,ID; XX,YY,CC,ATTR,ID;"), 5, 0, 1, 7)
        vm_group.setLayout(vm_layout)
        main_layout.addWidget(vm_group)

        # ---- 机器人实时状态（精简） ----
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
        status_layout.addWidget(self.joint_label, 1, 0, 1, 4)
        status_layout.addWidget(self.cart_label, 2, 0, 1, 4)

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

        # ---- 使用 QTabWidget 分离视觉标定和码垛工艺 ----
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        # ---------- 标签1：视觉标定 ----------
        calib_tab = QWidget()
        calib_layout = QVBoxLayout(calib_tab)
        calib_group = QGroupBox("视觉标定 (动态J2)")
        calib_inner = QGridLayout()
        calib_inner.addWidget(QLabel("视觉工艺号:"), 0, 0)
        self.vision_index_combo = QComboBox()
        self.vision_index_combo.addItems([f"VISION {i}" for i in range(8)])
        self.vision_index_combo.setCurrentIndex(0)
        calib_inner.addWidget(self.vision_index_combo, 0, 1)
        calib_inner.addWidget(QLabel("Mark点间距(mm):"), 0, 2)
        self.mark_distance_edit = QLineEdit("10")
        self.mark_distance_edit.setFixedWidth(60)
        calib_inner.addWidget(self.mark_distance_edit, 0, 3)
        calib_inner.addWidget(QLabel("步长(mm):"), 0, 4)
        self.step_size_edit = QLineEdit("20")
        self.step_size_edit.setFixedWidth(60)
        calib_inner.addWidget(self.step_size_edit, 0, 5)
        calib_inner.addWidget(QLabel("像素U,V:"), 0, 6)
        self.pixel_u_edit = QLineEdit("2592")
        self.pixel_u_edit.setFixedWidth(60)
        self.pixel_v_edit = QLineEdit("2048")
        self.pixel_v_edit.setFixedWidth(60)
        calib_inner.addWidget(self.pixel_u_edit, 0, 7)
        calib_inner.addWidget(self.pixel_v_edit, 0, 8)

        self.base_point_label = QLabel("基准点: 未记录 (需要 X,Y,Z,A,B,C)")
        calib_inner.addWidget(self.base_point_label, 1, 0, 1, 5)
        self.record_base_btn = QPushButton("记录基准点")
        self.record_base_btn.clicked.connect(self.on_record_base_point)
        calib_inner.addWidget(self.record_base_btn, 1, 5, 1, 2)

        self.apply_config_btn = QPushButton("① 写入配置")
        self.apply_config_btn.clicked.connect(self.on_apply_config)
        self.start_calib_btn = QPushButton("② 开始标定")
        self.start_calib_btn.clicked.connect(self.on_start_calib)
        self.verify_btn = QPushButton("③ 标定验证")
        self.verify_btn.clicked.connect(self.on_verify)
        self.diagnose_btn = QPushButton("🔧 诊断配置")
        self.diagnose_btn.clicked.connect(self.on_diagnose)
        calib_inner.addWidget(self.apply_config_btn, 2, 0)
        calib_inner.addWidget(self.start_calib_btn, 2, 1)
        calib_inner.addWidget(self.verify_btn, 2, 2)
        calib_inner.addWidget(self.diagnose_btn, 2, 3)

        self.calib_result_label = QLabel("标定状态: 等待操作")
        self.calib_result_label.setStyleSheet("background-color: #f0f0f0; padding: 5px;")
        calib_inner.addWidget(self.calib_result_label, 3, 0, 1, 6)

        calib_inner.addWidget(QLabel("视觉数据 (完整列表):"), 4, 0, 1, 2)
        self.vm_data_list = QListWidget()
        self.vm_data_list.setMaximumHeight(120)
        calib_inner.addWidget(self.vm_data_list, 5, 0, 1, 9)

        self.verify_result_label = QLabel("验证结果: 等待验证")
        calib_inner.addWidget(self.verify_result_label, 6, 0, 1, 6)
        calib_group.setLayout(calib_inner)
        calib_layout.addWidget(calib_group)
        calib_layout.addStretch()
        self.tab_widget.addTab(calib_tab, "视觉标定")

        # ---------- 标签2：码垛工艺 ----------
        pallet_tab = QWidget()
        pallet_layout = QVBoxLayout(pallet_tab)
        pallet_group = QGroupBox("码垛工艺配置 (种类1 四边形堆叠)")
        pallet_inner = QGridLayout()

        # 第一行：工艺号、种类
        pallet_inner.addWidget(QLabel("码垛工艺号:"), 0, 0)
        self.pallet_id_combo = QComboBox()
        self.pallet_id_combo.addItems([f"Pallet{i}" for i in range(16)])
        self.pallet_id_combo.setCurrentIndex(0)
        pallet_inner.addWidget(self.pallet_id_combo, 0, 1)
        pallet_inner.addWidget(QLabel("种类:"), 0, 2)
        self.pallet_type_combo = QComboBox()
        self.pallet_type_combo.addItems(["种类1(矩阵)", "种类2(自由)", "种类3(复合)"])
        self.pallet_type_combo.setCurrentIndex(0)
        self.pallet_type_combo.currentIndexChanged.connect(self.on_pallet_type_changed)
        pallet_inner.addWidget(self.pallet_type_combo, 0, 3)

        # 第二行：参考点记录
        pallet_inner.addWidget(QLabel("参考点 P1 (PR10):"), 1, 0)
        self.p1_record_btn = QPushButton("记录P1")
        self.p1_record_btn.clicked.connect(lambda: self.record_pr_point(10, "P1"))
        pallet_inner.addWidget(self.p1_record_btn, 1, 1)
        self.p1_label = QLabel("未记录")
        pallet_inner.addWidget(self.p1_label, 1, 2)
        pallet_inner.addWidget(QLabel("行上点 P2 (PR11):"), 1, 3)
        self.p2_record_btn = QPushButton("记录P2")
        self.p2_record_btn.clicked.connect(lambda: self.record_pr_point(11, "P2"))
        pallet_inner.addWidget(self.p2_record_btn, 1, 4)
        self.p2_label = QLabel("未记录")
        pallet_inner.addWidget(self.p2_label, 1, 5)
        pallet_inner.addWidget(QLabel("列上点 P3 (PR12):"), 1, 6)
        self.p3_record_btn = QPushButton("记录P3")
        self.p3_record_btn.clicked.connect(lambda: self.record_pr_point(12, "P3"))
        pallet_inner.addWidget(self.p3_record_btn, 1, 7)
        self.p3_label = QLabel("未记录")
        pallet_inner.addWidget(self.p3_label, 1, 8)

        # 第三行：行列层参数
        pallet_inner.addWidget(QLabel("行数:"), 2, 0)
        self.row_edit = QLineEdit("3")
        self.row_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.row_edit, 2, 1)
        pallet_inner.addWidget(QLabel("列数:"), 2, 2)
        self.col_edit = QLineEdit("4")
        self.col_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.col_edit, 2, 3)
        pallet_inner.addWidget(QLabel("层数:"), 2, 4)
        self.layer_edit = QLineEdit("4")
        self.layer_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.layer_edit, 2, 5)
        pallet_inner.addWidget(QLabel("层高(mm):"), 2, 6)
        self.height_edit = QLineEdit("20")
        self.height_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.height_edit, 2, 7)

        # 第四行：顺序
        pallet_inner.addWidget(QLabel("码垛顺序:"), 3, 0)
        self.order_combo = QComboBox()
        orders = ["行-列-层", "行-层-列", "列-行-层", "列-层-行",
                  "层-行-列", "层-列-行", "S型行排序", "S型列排序"]
        self.order_combo.addItems(orders)
        self.order_combo.setCurrentIndex(0)
        pallet_inner.addWidget(self.order_combo, 3, 1, 1, 2)

        # 第五行：趋近点/离开点
        pallet_inner.addWidget(QLabel("趋近点Z模式:"), 4, 0)
        self.approach_z_mode = QComboBox()
        self.approach_z_mode.addItems(["Z增量", "Z不变"])
        pallet_inner.addWidget(self.approach_z_mode, 4, 1)
        pallet_inner.addWidget(QLabel("偏移(mm):"), 4, 2)
        self.approach_z_offset = QLineEdit("50")
        self.approach_z_offset.setFixedWidth(60)
        pallet_inner.addWidget(self.approach_z_offset, 4, 3)
        pallet_inner.addWidget(QLabel("离开点Z模式:"), 4, 4)
        self.depart_z_mode = QComboBox()
        self.depart_z_mode.addItems(["Z增量", "Z不变"])
        pallet_inner.addWidget(self.depart_z_mode, 4, 5)
        pallet_inner.addWidget(QLabel("偏移(mm):"), 4, 6)
        self.depart_z_offset = QLineEdit("50")
        self.depart_z_offset.setFixedWidth(60)
        pallet_inner.addWidget(self.depart_z_offset, 4, 7)

        # 第六行：操作按钮
        self.setup_pallet_btn = QPushButton("设置码垛参数")
        self.setup_pallet_btn.clicked.connect(self.on_setup_pallet)
        self.update_pos_btn = QPushButton("更新码垛位置")
        self.update_pos_btn.clicked.connect(self.on_update_pallet_pos)
        self.move_target_btn = QPushButton("移动到堆叠点")
        self.move_target_btn.clicked.connect(lambda: self.on_move_pallet("targetpos"))
        self.move_pre_btn = QPushButton("移动到趋近点")
        self.move_pre_btn.clicked.connect(lambda: self.on_move_pallet("prepos"))
        self.move_post_btn = QPushButton("移动到离开点")
        self.move_post_btn.clicked.connect(lambda: self.on_move_pallet("postpos"))
        self.update_plr_btn = QPushButton("更新PLR (下一个点)")
        self.update_plr_btn.clicked.connect(self.on_update_plr)
        pallet_inner.addWidget(self.setup_pallet_btn, 5, 0, 1, 2)
        pallet_inner.addWidget(self.update_pos_btn, 5, 2, 1, 2)
        pallet_inner.addWidget(self.move_target_btn, 5, 4)
        pallet_inner.addWidget(self.move_pre_btn, 5, 5)
        pallet_inner.addWidget(self.move_post_btn, 5, 6)
        pallet_inner.addWidget(self.update_plr_btn, 5, 7)

        # 第七行：显示当前 PLR
        pallet_inner.addWidget(QLabel("当前PLR (行列层):"), 6, 0)
        self.plr_display = QLabel("--")
        pallet_inner.addWidget(self.plr_display, 6, 1, 1, 2)

        # 提示
        pallet_inner.addWidget(QLabel("提示: 使用前请确保已示教P1,P2,P3点，并切换至自动模式/上使能"), 7, 0, 1, 8)

        pallet_group.setLayout(pallet_inner)
        pallet_layout.addWidget(pallet_group)
        pallet_layout.addStretch()
        self.tab_widget.addTab(pallet_tab, "码垛工艺")

        # ---- 日志 ----
        log_group = QGroupBox("日志")
        log_layout = QVBoxLayout()
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

        # 初始化使能
        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    # ===== 辅助方法 =====
    def _set_robot_controls_enabled(self, enabled):
        for w in [self.movj_btn, self.movl_btn, self.emergency_btn,
                  self.speed_slider, self.mode_auto_btn, self.mode_autocmd_btn,
                  self.mode_manual_btn, self.servo_on_btn, self.servo_off_btn,
                  self.reset_alarm_btn, self.switch_uf_btn, self.switch_tf_btn,
                  self.p1_record_btn, self.p2_record_btn, self.p3_record_btn,
                  self.setup_pallet_btn, self.update_pos_btn, self.move_target_btn,
                  self.move_pre_btn, self.move_post_btn, self.update_plr_btn]:
            w.setEnabled(enabled)

    def _set_vm_controls_enabled(self, enabled):
        self.vm_disconnect_btn.setEnabled(enabled)
        self.vm_trigger_btn.setEnabled(enabled)

    def _set_calib_controls_enabled(self, enabled):
        for w in [self.record_base_btn, self.apply_config_btn,
                  self.start_calib_btn, self.verify_btn, self.diagnose_btn]:
            w.setEnabled(enabled)

    # ===== 槽函数 =====
    # ---- 机器人 ----
    def on_robot_connect(self):
        ip = self.robot_ip_edit.text().strip()
        if not ip:
            self.append_log("[错误] 请输入机器人IP")
            return
        self.append_log(f"正在连接机器人 {ip} ...")
        self.robot_worker.set_ip(ip)
        self.robot_worker.connect_robot()

    def on_robot_disconnect(self):
        self.robot_worker.disconnect_robot()
        self.state_timer.stop()
        self.is_robot_connected = False
        self.robot_connect_btn.setEnabled(True)
        self.robot_disconnect_btn.setEnabled(False)
        self.robot_status_label.setText("⚪ 未连接")
        self._set_robot_controls_enabled(False)
        self._set_calib_controls_enabled(False)

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

    def request_state(self):
        self.robot_worker.get_state()

    def update_state(self, info):
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
        uf_no = info['uf_no']
        tf_no = info['tf_no']
        self.current_uf_label.setText(f"UF{uf_no}")
        self.current_tf_label.setText(f"TF{tf_no}")
        ufp = info['uf_pose']
        tfp = info['tf_pose']
        self.uf_pose_label.setText(
            f"UF{uf_no}位姿: X={ufp.x:.2f} Y={ufp.y:.2f} Z={ufp.z:.2f} "
            f"A={ufp.a:.2f} B={ufp.b:.2f} C={ufp.c:.2f}"
        )
        self.tf_pose_label.setText(
            f"TF{tf_no}位姿: X={tfp.x:.2f} Y={tfp.y:.2f} Z={tfp.z:.2f} "
            f"A={tfp.a:.2f} B={tfp.b:.2f} C={tfp.c:.2f}"
        )

    def on_speed_changed(self, value):
        self.speed_value_label.setText(f"{value}%")
        self.robot_worker.set_speed(value)

    def on_movj(self):
        self.robot_worker.movj_to(10, 0, 0, 0, 0, 0)

    def on_movl(self):
        self.robot_worker.movl_to(300, 0, 100)

    # ---- UF/TF切换 ----
    def on_switch_uf(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        uf_text = self.switch_uf_combo.currentText()
        uf_index = int(uf_text.replace("UF", ""))
        self.robot_worker.switch_uf(uf_index)

    def on_switch_tf(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        tf_text = self.switch_tf_combo.currentText()
        tf_index = int(tf_text.replace("TF", ""))
        self.robot_worker.switch_tf(tf_index)

    # ---- Vision Master ----
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
        self.vm_thread.connect_to_server()

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
        pass

    def on_vm_data_received(self, points):
        self.latest_vm_points = points
        self.vm_data_list.clear()
        attrs = set([p['attr'] for p in points])
        self.append_log(f"[VM] ATTR 值统计: {sorted(attrs)}")
        if 0 in attrs and 1 in attrs and 2 in attrs and 3 in attrs:
            self.append_log("[VM] ✅ ATTR 包含 0,1,2,3 识别正确")
        else:
            self.append_log("[VM] ⚠️ ATTR 缺少部分值，标定可能失败！")
        for i, p in enumerate(points):
            item_text = (f"{i+1:2d}. X={p['x']:8.3f}  Y={p['y']:8.3f}  "
                        f"C={p['c']:7.3f}  ATTR={p['attr']}  ID={p['id']}")
            self.vm_data_list.addItem(item_text)
        self.append_log(f"[VM] 收到 {len(points)} 个点")

    def on_trigger_vm(self):
        if not self.is_vm_connected:
            self.append_log("[错误] Vision Master 未连接")
            return
        self.append_log("[VM] 触发拍照...")
        self.vm_data_list.clear()
        self.vm_thread.trigger_and_get()

    # ---- 标定 ----
    def get_current_pose(self):
        if self.robot_worker.handle is None:
            return None
        try:
            point = x5.get_cpoint(self.robot_worker.handle)
            return (point.pose.x, point.pose.y, point.pose.z,
                    point.pose.a, point.pose.b, point.pose.c,
                    point.uf, point.tf, point.cfg)
        except:
            return None

    def on_record_base_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        uf = pose[6]
        tf = pose[7]
        cfg = pose[8] if len(pose) > 8 else (0,0,0,1)
        p = pose[:6]
        pose_obj = x5.Pose(p[0], p[1], p[2], p[3], p[4], p[5], 0,0,0)
        point_obj = x5.Point(pose=pose_obj, uf=uf, tf=tf, cfg=cfg)
        self.base_point = point_obj
        self.base_point_label.setText(
            f"基准点: X={p[0]:.2f} Y={p[1]:.2f} Z={p[2]:.2f} "
            f"A={p[3]:.2f} B={p[4]:.2f} C={p[5]:.2f} (UF{uf}, TF{tf})"
        )
        self.append_log(f"[基准点] 已记录 Point 对象")

    def on_apply_config(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.base_point is None:
            self.append_log("[错误] 请先记录基准点")
            return

        if self.is_vm_connected:
            self.append_log("[预检查] 触发拍照验证视觉数据...")
            import threading
            event = threading.Event()
            received_points = []

            def on_data(points):
                received_points.extend(points)
                event.set()

            self.vm_thread.data_received.connect(on_data)
            self.vm_thread.trigger_and_get()
            if not event.wait(5.0):
                self.append_log("[预检查] 触发拍照超时，请检查视觉连接")
                self.vm_thread.data_received.disconnect(on_data)
                return
            self.vm_thread.data_received.disconnect(on_data)

            if not received_points:
                self.append_log("[预检查] 未收到任何视觉数据")
                return
            attrs = set([p['attr'] for p in received_points])
            if not (0 in attrs and 1 in attrs and 2 in attrs and 3 in attrs):
                self.append_log(f"[预检查] ATTR不完整，当前ATTR: {sorted(attrs)}")
                return
            y_values = [p['y'] for p in received_points]
            if all(y == 0.0 for y in y_values):
                self.append_log("[预检查] 警告：所有点的Y坐标均为0")
            else:
                self.append_log("[预检查] 视觉数据检查通过")
        else:
            self.append_log("[预检查] 视觉未连接，跳过")

        try:
            vision_idx = int(self.vision_index_combo.currentText().split()[-1])
            protocol = 0 if self.vm_protocol_combo.currentIndex() == 0 else 1
            trigger_type = 0 if self.vm_trigger_type_combo.currentIndex() == 0 else 1
            calib_object = 0 if self.vm_calib_object_combo.currentIndex() == 0 else 1
            calib_type = 0 if self.vm_calib_type_combo.currentIndex() == 0 else 1
            point_count = self.vm_calib_points_combo.currentIndex()
            mount_type = 2
            uf_index = self.vm_uf_combo.currentIndex()
            tf_index = self.vm_tf_combo.currentIndex()
            invert_c = self.vm_c_invert_check.isChecked()

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

            calib_process = x5v.VisionCalibProcess()
            calib_process.calibration_type = calib_type
            calib_process.point_count = point_count
            calib_process.is_workplane_parallel = True

            pixel_u = int(self.pixel_u_edit.text())
            pixel_v = int(self.pixel_v_edit.text())
            mark_dist = int(self.mark_distance_edit.text())
            step = int(self.step_size_edit.text())

            calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
                tf_index=tf_index,
                pixel_shift=[pixel_u, pixel_v],
                error=0.0,
                base_point=self.base_point,
                mark_distance=mark_dist,
                step_size=step
            )

            self.robot_worker.apply_vision_config(vision_idx, config, calib_process)
            self.calib_result_label.setText("标定状态: ✅ 配置已写入")
            self.calib_result_label.setStyleSheet("background-color: #d4edda; padding: 5px;")
            self.append_log("✅ Vision配置和标定参数写入成功")
        except Exception as e:
            self.append_log(f"[配置] 异常: {str(e)}")
            traceback.print_exc()
            self.calib_result_label.setText("标定状态: ❌ 配置异常")
            self.calib_result_label.setStyleSheet("background-color: #f8d7da; padding: 5px;")

    def on_start_calib(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        vision_idx = int(self.vision_index_combo.currentText().split()[-1])
        self.append_log("="*60)
        self.append_log("[标定] 开始自动标定...")
        self.append_log("[标定] 确保标定板在视野内")
        self.append_log("="*60)
        self.calib_result_label.setText("标定状态: 进行中... ⏳")
        self.calib_result_label.setStyleSheet("background-color: #fff3cd; padding: 5px;")
        self.robot_worker.run_auto_calib(vision_idx)

    def on_calib_result(self, flag, errors, transformation):
        if flag == 1:
            self.append_log("[标定] ✅ 成功")
            self.append_log(f"[标定] X平均误差: {errors[0]:.4f} mm, Y平均: {errors[1]:.4f} mm")
            vision_idx = int(self.vision_index_combo.currentText().split()[-1])
            try:
                x5v.vision_set_calib_par(self.robot_worker.handle, vision_idx, transformation)
                self.append_log("[标定] 参数已自动应用")
                self.calib_result_label.setText(f"标定状态: ✅ 成功 (X={errors[0]:.3f}mm Y={errors[1]:.3f}mm)")
                self.calib_result_label.setStyleSheet("background-color: #d4edda; padding: 5px;")
            except Exception as e:
                self.append_log(f"[标定] 应用失败: {e}")
                self.calib_result_label.setText("标定状态: ✅ 标定完成但应用参数失败")
                self.calib_result_label.setStyleSheet("background-color: #fff3cd; padding: 5px;")
        elif flag == -1:
            self.append_log("[标定] ⚠️ 完成但误差超标")
            self.calib_result_label.setText("标定状态: ⚠️ 误差超标")
            self.calib_result_label.setStyleSheet("background-color: #fff3cd; padding: 5px;")
        else:
            self.append_log("[标定] ❌ 失败")
            self.calib_result_label.setText("标定状态: ❌ 失败")
            self.calib_result_label.setStyleSheet("background-color: #f8d7da; padding: 5px;")

    def on_verify(self):
        if not self.is_robot_connected or not self.latest_vm_points:
            self.append_log("[错误] 无视觉数据或机器人未连接")
            return
        p = self.latest_vm_points[0]
        vision_idx = int(self.vision_index_combo.currentText().split()[-1])
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取机器人位置失败")
            return
        trig_pose = x5.Pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], 0,0,0)
        trig_point = x5.Point(pose=trig_pose, uf=pose[6], tf=pose[7], cfg=pose[8] if len(pose)>8 else (0,0,0,1))
        pixel_pose = x5.Pose(p['x'], p['y'], 0, 0, 0, p['c'], 0,0,0)
        self.append_log(f"[验证] 像素点: X={p['x']:.2f} Y={p['y']:.2f} C={p['c']:.2f}")
        self.robot_worker.verify_vision(vision_idx, pixel_pose, trig_point)

    def on_verify_result(self, result_pose, error):
        if error:
            self.verify_result_label.setText(f"验证结果: ❌ {error}")
            self.append_log(f"[验证] 失败: {error}")
        else:
            self.verify_result_label.setText(
                f"验证结果: 机器人坐标 X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}"
            )
            self.append_log(f"[验证] ✅ 转换后: X={result_pose.x:.2f} Y={result_pose.y:.2f} C={result_pose.c:.2f}")

    def on_diagnose(self):
        self.append_log("="*60)
        self.append_log("[诊断] 检查配置...")
        if not self.is_robot_connected:
            self.append_log("[诊断] ❌ 机器人未连接")
            return
        self.append_log(f"[诊断] ✅ 机器人已连接")
        if not self.is_vm_connected:
            self.append_log("[诊断] ❌ Vision Master 未连接")
            return
        self.append_log("[诊断] ✅ Vision Master 已连接")
        if not self.latest_vm_points:
            self.append_log("[诊断] ❌ 无视觉数据")
        else:
            attrs = set([p['attr'] for p in self.latest_vm_points])
            self.append_log(f"[诊断] ATTR: {sorted(attrs)}")
        if self.base_point is None:
            self.append_log("[诊断] ❌ 未记录基准点")
        else:
            self.append_log("[诊断] ✅ 基准点已记录")
        self.append_log("="*60)

    # ===== 码垛功能 =====
    def on_pallet_type_changed(self, index):
        # 仅支持种类1
        if index != 0:
            QMessageBox.warning(self, "提示", "目前仅支持种类1（四边形堆叠），其他种类暂未实现")
            self.pallet_type_combo.setCurrentIndex(0)

    def record_pr_point(self, pr_index, name):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        uf = pose[6]
        tf = pose[7]
        cfg = pose[8] if len(pose) > 8 else (0,0,0,1)
        p = pose[:6]
        pose_obj = x5.Pose(p[0], p[1], p[2], p[3], p[4], p[5], 0,0,0)
        point_obj = x5.Point(pose=pose_obj, uf=uf, tf=tf, cfg=cfg)
        self.robot_worker.set_pr_value(pr_index, point_obj)
        # 更新标签显示
        label_text = f"X={p[0]:.1f} Y={p[1]:.1f} Z={p[2]:.1f} C={p[5]:.1f}"
        if name == "P1":
            self.p1_label.setText(label_text)
        elif name == "P2":
            self.p2_label.setText(label_text)
        elif name == "P3":
            self.p3_label.setText(label_text)
        self.append_log(f"[码垛] {name} (PR{pr_index}) 已记录")

    def on_setup_pallet(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pallet_id = self.pallet_id_combo.currentIndex()
        pallet_type = self.pallet_type_combo.currentIndex() + 1
        if pallet_type != 1:
            QMessageBox.warning(self, "提示", "目前仅支持种类1")
            return
        # 检查是否已记录P1,P2,P3
        if self.p1_label.text() == "未记录" or self.p2_label.text() == "未记录" or self.p3_label.text() == "未记录":
            QMessageBox.warning(self, "提示", "请先记录P1,P2,P3点")
            return
        try:
            row = int(self.row_edit.text())
            col = int(self.col_edit.text())
            layer = int(self.layer_edit.text())
            height = float(self.height_edit.text())
        except ValueError:
            self.append_log("[错误] 行列层或层高输入无效")
            return
        order_idx = self.order_combo.currentIndex()
        # 将顺序索引映射为字符串，用于 Pallet.Load 的最后一个参数（可忽略，因为 Pallet.Load 没有顺序参数，顺序在后续设置中可能需要额外指令）
        # 但 Pallet.Load 只有 (pallet_id, p1, p2, p3, row, col, layer, height)，没有顺序，顺序通过其他方式设置？
        # 说明书中的顺序是在参数设置中配置的，但 Pallet.Load 似乎不包含顺序。
        # 我们暂时忽略顺序，后续可以通过其他指令设置。
        approach_z_mode = self.approach_z_mode.currentIndex()  # 0增量，1不变
        approach_z_offset = float(self.approach_z_offset.text())
        depart_z_mode = self.depart_z_mode.currentIndex()
        depart_z_offset = float(self.depart_z_offset.text())

        self.robot_worker.setup_pallet(pallet_id, pallet_type, 10, 11, 12,
                                       row, col, layer, height, order_idx,
                                       approach_z_mode, approach_z_offset,
                                       depart_z_mode, depart_z_offset)

    def on_update_pallet_pos(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pallet_id = self.pallet_id_combo.currentIndex()
        self.robot_worker.update_pallet_pos(pallet_id)

    def on_move_pallet(self, target):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pallet_id = self.pallet_id_combo.currentIndex()
        self.robot_worker.move_pallet(pallet_id, target)

    def on_update_plr(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pallet_id = self.pallet_id_combo.currentIndex()
        self.robot_worker.update_plr(pallet_id)

    # ===== 日志 =====
    def append_log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{ts}] {msg}")
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    def clear_log(self):
        self.log_text.clear()

    def closeEvent(self, event):
        self.state_timer.stop()
        self.vm_thread.disconnect()
        self.vm_thread.quit()
        self.vm_thread.wait()
        self.robot_worker.quit()
        self.robot_worker.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())