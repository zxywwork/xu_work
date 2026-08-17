import sys
import socket
import time
import traceback
import math
import os
import json
from datetime import datetime
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt, QTimer, QMetaObject, Q_ARG
from PyQt5.QtGui import QFont, QColor

import xapi.api as x5
import xapi.api.vision as x5v

from threading import Lock
# 全局串行锁：x5/x5v 原生 SDK 非线程安全，所有线程（主线程、机器人线程、
# 连续运行线程、视觉码垛线程）的 SDK 调用都必须经过它，避免并发调用导致闪退。
X5_LOCK = Lock()


# ============================================================
# 工作线程1：Vision Master 通信（含同步触发方法）
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
        self.trigger_cmd = "trigger"
        self.timeout = 3.0
        self._trigger_flag = False
        self._stop_flag = False
        from threading import Lock
        self.sock_lock = Lock()  # 保护 socket 收发，避免多线程同时读写

    def set_params(self, ip, port, trigger_cmd="trigger", timeout=3.0):
        self.ip = ip
        self.port = port
        self.trigger_cmd = trigger_cmd
        self.timeout = timeout

    def connect_to_server(self):
        self._stop_flag = False
        if not self.isRunning():
            self.start()
        self._trigger_flag = "connect"

    def disconnect(self):
        self._trigger_flag = False
        if self.sock:
            with self.sock_lock:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
        self.connected_signal.emit(False)

    def stop(self):
        self._stop_flag = True
        self.disconnect()

    def trigger_and_get(self):
        if self.sock is None:
            self.log_signal.emit("[VM] 错误: 未连接")
            return
        self._trigger_flag = "trigger"

    def trigger_and_get_sync(self, timeout=None):
        """同步触发拍照并获取数据，直接返回点列表（用于子线程）"""
        with self.sock_lock:
            if self.sock is None:
                self.log_signal.emit("[VM] 错误: 未连接")
                return []
            try:
                if timeout is not None:
                    self.sock.settimeout(timeout)
                self.sock.send(self.trigger_cmd.encode())
                self.log_signal.emit(f"[VM] 发送触发: {self.trigger_cmd}")
                raw_data = self.sock.recv(4096).decode().strip()
                self.raw_data_received.emit(raw_data)
                points = self._parse_data(raw_data)
                if points:
                    self.log_signal.emit(f"[VM] 解析到 {len(points)} 个点")
                    self.data_received.emit(points)
                else:
                    self.log_signal.emit("[VM] 警告: 数据解析为空")
                return points
            except socket.timeout:
                self.log_signal.emit("[VM] 超时: 等待响应超时")
                return []
            except Exception as e:
                self.log_signal.emit(f"[VM] 错误: {str(e)}")
                return []

    def run(self):
        while not self._stop_flag:
            if self._trigger_flag == "connect":
                self._trigger_flag = False
                self._do_connect()
            elif self._trigger_flag == "trigger":
                self._trigger_flag = False
                self._do_trigger()
            self.msleep(50)
        self.disconnect()

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
        with self.sock_lock:
            if self.sock is None:
                self.log_signal.emit("[VM] 错误: 未连接")
                return
            try:
                self.sock.send(self.trigger_cmd.encode())
                self.log_signal.emit(f"[VM] 发送触发: {self.trigger_cmd}")
                self.sock.settimeout(self.timeout)
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
# 工作线程2：机器人 API 调用
# ============================================================
class RobotWorker(QThread):
    state_signal = pyqtSignal(dict)
    log_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)
    calib_result_signal = pyqtSignal(int, list, list)
    verify_result_signal = pyqtSignal(object, str)
    connection_lost_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.handle = None
        self.ip = ""
        self.running = False
        self.cmd_queue = []
        from threading import Lock
        self.cmd_lock = Lock()
        self._stop_flag = False

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

    def abort_immediately(self, lock_motion=True):
        with self.cmd_lock:
            self.cmd_queue.clear()
            if lock_motion:
                self._stop_flag = True
        if self.handle is not None:
            try:
                x5.abort(self.handle)
                self.log_signal.emit("[急停] 机器人已停止，命令队列已清空")
            except Exception as e:
                self.log_signal.emit(f"[急停] 失败: {str(e)}")
        else:
            self.log_signal.emit("[急停] 机器人未连接，仅清空命令队列")

    def set_speed(self, speed):
        with self.cmd_lock:
            self.cmd_queue.append(("set_speed", speed))

    def movj_to(self, j1=0, j2=0, j3=0, j4=0, j5=0, j6=0):
        with self.cmd_lock:
            self.cmd_queue.append(("movj", (j1, j2, j3, j4, j5, j6)))

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

    def set_pr_value(self, pr_index, point):
        with self.cmd_lock:
            self.cmd_queue.append(("set_pr", pr_index, point))

    def movj_to_pose(self, pose, uf, tf, cfg):
        with self.cmd_lock:
            self.cmd_queue.append(("movj_to_pose", pose, uf, tf, cfg))

    def movl_to_pose(self, pose, uf, tf, cfg):
        with self.cmd_lock:
            self.cmd_queue.append(("movl_to_pose", pose, uf, tf, cfg))

    def wait_idle(self, timeout=25.0, target=None, tolerance=5.0, stop_check=None):
        """等待命令队列清空且机器人到位。返回 (是否到位, 原因/说明)。

        视觉码垛等流程中调用：确保机器人真正到达目标点后再进行拍照/抓取。
        - target 传入期望位姿(Pose/Point)时，会同时校验当前位置与目标的
          平面距离 <= tolerance(mm) 且 Z 偏差 <= tolerance(mm)，避免
          “机器人还没来得及启动就误判已到位”的竞态。
        - stop_check 为可选的停止检测回调（返回 True 时立即结束等待并返回 False）。
        - 等待期间检测到机器人报警会立即返回 False 并说明报警代码；
          超时会附带当前实际位置与目标的偏差，方便定位原因。
        """
        start = time.time()
        last_state = None
        while time.time() - start < timeout:
            if stop_check is not None and stop_check():
                return (False, "用户停止")
            with self.cmd_lock:
                idle = not self.cmd_queue
            if self.handle is None:
                return (False, "机器人未连接")
            try:
                with X5_LOCK:
                    state = x5.get_system_state(self.handle)
                last_state = state
                if state.alarm:
                    return (False, f"机器人报警(代码{state.alarm})，请先复位报警")
                if idle and state.in_pos:
                    if target is None:
                        return (True, "到位")
                    with X5_LOCK:
                        cp = x5.get_cpoint(self.handle)
                    pose = cp.pose
                    dx = pose.x - target.x
                    dy = pose.y - target.y
                    dz = pose.z - target.z
                    if math.hypot(dx, dy) <= tolerance and abs(dz) <= tolerance:
                        return (True, "到位")
            except Exception:
                time.sleep(0.1)
                continue
            time.sleep(0.1)

        reason = f"等待到位超时({timeout:.0f}s)"
        if last_state is not None:
            reason += f" | in_pos={last_state.in_pos} alarm={last_state.alarm}"
        if target is not None:
            try:
                with X5_LOCK:
                    cp = x5.get_cpoint(self.handle)
                pose = cp.pose
                dx = pose.x - target.x
                dy = pose.y - target.y
                dz = pose.z - target.z
                reason += (f" | 当前位置({pose.x:.1f},{pose.y:.1f},{pose.z:.1f}) "
                           f"目标({target.x:.1f},{target.y:.1f},{target.z:.1f}) "
                           f"偏差({math.hypot(dx, dy):.1f},{abs(dz):.1f})mm")
            except Exception:
                pass
        return (False, reason)

    def stop(self):
        self.running = False
        with self.cmd_lock:
            self.cmd_queue.clear()

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
                if (self._stop_flag and isinstance(cmd, tuple)
                        and cmd[0] in ("movj", "movj_to_pose", "movl_to_pose")):
                    self.log_signal.emit("[急停] 已忽略运动命令，请先复位报警")
                    continue
                with X5_LOCK:
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
                    elif cmd[0] == "set_speed":
                        self._do_set_speed(cmd[1])
                    elif cmd[0] == "movj":
                        self._do_movj(*cmd[1])
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
                    elif cmd[0] == "set_pr":
                        self._do_set_pr(cmd[1], cmd[2])
                    elif cmd[0] == "movj_to_pose":
                        self._do_movj_to_pose(cmd[1], cmd[2], cmd[3], cmd[4])
                    elif cmd[0] == "movl_to_pose":
                        self._do_movl_to_pose(cmd[1], cmd[2], cmd[3], cmd[4])
                    else:
                        self.log_signal.emit(f"[Worker] 未知命令: {cmd}")
            except Exception as e:
                self.log_signal.emit(f"[Worker] 命令执行异常: {str(e)}")
                traceback.print_exc()

    def _do_connect(self):
        try:
            self.handle = x5.connect(self.ip)
            self._stop_flag = False
            self.connected_signal.emit(True)
            self.log_signal.emit(f"[机器人] 连接成功，句柄: {self.handle}")
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
                self.log_signal.emit("[状态获取] 检测到网络断开，请求自动重连...")
                self.handle = None
                self.connected_signal.emit(False)
                self.connection_lost_signal.emit()
            else:
                self.log_signal.emit(f"[状态获取异常] {err_str}")

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
            self._stop_flag = False
            self.log_signal.emit("[报警复位] 成功，运动命令已解锁")
        except Exception as e:
            self.log_signal.emit(f"[报警复位] 失败: {str(e)}")

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
            result_pose = x5.vision_dynamic_cnvrt(self.handle, vision_idx, pixel_pose, trig_point)
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

    def _do_set_pr(self, pr_index, point):
        if self.handle is None:
            return
        try:
            x5.set_pr(self.handle, pr_index, point)
            self.log_signal.emit(f"[PR] PR{pr_index} 已更新")
        except Exception as e:
            self.log_signal.emit(f"[PR] 设置 PR{pr_index} 失败: {str(e)}")

    def _do_movj_to_pose(self, pose, uf, tf, cfg):
        if self.handle is None:
            self.log_signal.emit("[MOVJ] 机器人未连接")
            return
        try:
            point = x5.Point(pose=pose, uf=uf, tf=tf, cfg=cfg)
            x5.movj(self.handle, point)
            self.log_signal.emit(f"[MOVJ] 到 ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[MOVJ] 失败: {str(e)}")

    def _do_movl_to_pose(self, pose, uf, tf, cfg):
        if self.handle is None:
            self.log_signal.emit("[MOVL] 机器人未连接")
            return
        try:
            point = x5.Point(pose=pose, uf=uf, tf=tf, cfg=cfg)
            x5.movl(self.handle, point)
            self.log_signal.emit(f"[MOVL] 到 ({pose.x:.1f}, {pose.y:.1f}, {pose.z:.1f})")
        except Exception as e:
            self.log_signal.emit(f"[MOVL] 失败: {str(e)}")

# ============================================================
# 主窗口
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("视觉码垛控制系统 - 完整版（点位可编辑）")
        self.setGeometry(50, 50, 1800, 1300)

        # ---- 创建日志文件 ----
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.log_file_path = os.path.join(log_dir, f"vision_pallet_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
        with open(self.log_file_path, 'w', encoding='utf-8') as f:
            f.write(f"日志开始于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("="*60 + "\n")

        self.robot_worker = RobotWorker()
        self.robot_worker.connected_signal.connect(self.on_robot_connected)
        self.robot_worker.log_signal.connect(self.append_log)
        self.robot_worker.state_signal.connect(self.update_state)
        self.robot_worker.calib_result_signal.connect(self.on_calib_result)
        self.robot_worker.verify_result_signal.connect(self.on_verify_result)
        self.robot_worker.connection_lost_signal.connect(self.on_robot_connection_lost)
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

        # ---- 标定状态记录 ----
        self.calibration_status = {i: False for i in range(8)}
        self.calib_data = {i: None for i in range(8)}          # 各工艺标定变换矩阵 list[float](12)
        self.last_calib_params = {i: None for i in range(8)}   # 各工艺标定配置参数（用于保存）

        # ---- 码垛变量 ----
        self.pallet_points = []
        self.current_step = 0
        self.total_steps = 0
        self.p1_point = None
        self.p2_point = None
        self.p3_point = None
        self.safe_point = None
        self.photo_point = None
        self.pallet_uf = 0
        self.pallet_tf = 0
        self.pallet_cfg = (0, 0, 0, 1)
        self.selected_row = -1
        self.run_all_flag = False
        self.stop_requested = False
        self.vision_pallet_running = False
        self.vision_pallet_stop = False

        self.init_ui()

    def append_log(self, msg):
        # Qt 控件只能从主线程操作；若从工作线程调用，则投递回 GUI 线程执行
        if QThread.currentThread() is not self.thread():
            QMetaObject.invokeMethod(self, "_append_log_ui", Qt.QueuedConnection,
                                     Q_ARG(str, msg))
            return
        self._append_log_ui(msg)

    @pyqtSlot(str)
    def _append_log_ui(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        full_msg = f"[{ts}] {msg}"
        self.log_text.append(full_msg)
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
        try:
            with open(self.log_file_path, 'a', encoding='utf-8') as f:
                f.write(full_msg + "\n")
                f.flush()
        except Exception:
            pass

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(5, 5, 5, 5)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(5)

        # 机器人控制
        robot_group = QGroupBox("机器人控制")
        robot_layout = QVBoxLayout()
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

        row3 = QHBoxLayout()
        self.uf_pose_label = QLabel("UF位姿: --")
        self.tf_pose_label = QLabel("TF位姿: --")
        row3.addWidget(self.uf_pose_label)
        row3.addWidget(self.tf_pose_label)
        row3.addStretch()
        robot_layout.addLayout(row3)
        robot_group.setLayout(robot_layout)
        left_layout.addWidget(robot_group)

        # Vision Master
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
        left_layout.addWidget(vm_group)

        # 机器人状态
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

        self.movj_btn = QPushButton("执行 movj (关节零位)")
        self.movj_btn.clicked.connect(lambda: self.robot_worker.movj_to(0, 0, 0, 0, 0, 0))
        self.emergency_btn = QPushButton("🚨 急停")
        self.emergency_btn.setStyleSheet("background-color: #cc0000; color: white; font-weight: bold; font-size: 14px;")
        self.emergency_btn.clicked.connect(self.on_emergency_stop)
        status_layout.addWidget(self.movj_btn, 5, 0)
        status_layout.addWidget(self.emergency_btn, 5, 1, 1, 3)
        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)

        # 选项卡
        self.tab_widget = QTabWidget()
        left_layout.addWidget(self.tab_widget)

        # 视觉标定标签
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
        self.step_size_edit = QLineEdit("10")
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
        self.save_calib_btn = QPushButton("💾 保存标定")
        self.save_calib_btn.clicked.connect(self.on_save_calib)
        self.load_calib_btn = QPushButton("📂 导入标定")
        self.load_calib_btn.clicked.connect(self.on_load_calib)
        calib_inner.addWidget(self.apply_config_btn, 2, 0)
        calib_inner.addWidget(self.start_calib_btn, 2, 1)
        calib_inner.addWidget(self.verify_btn, 2, 2)
        calib_inner.addWidget(self.diagnose_btn, 2, 3)
        calib_inner.addWidget(self.save_calib_btn, 2, 4)
        calib_inner.addWidget(self.load_calib_btn, 2, 5)

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

        # 码垛标签
        pallet_tab = QWidget()
        pallet_layout = QVBoxLayout(pallet_tab)
        pallet_group = QGroupBox("手动码垛控制（点位数可编辑）")
        pallet_inner = QGridLayout()

        # 参考点记录
        pallet_inner.addWidget(QLabel("参考点 P1:"), 0, 0)
        self.p1_x_edit = QLineEdit("0")
        self.p1_y_edit = QLineEdit("0")
        self.p1_z_edit = QLineEdit("0")
        self.p1_c_edit = QLineEdit("0")
        self.p1_x_edit.setFixedWidth(60)
        self.p1_y_edit.setFixedWidth(60)
        self.p1_z_edit.setFixedWidth(60)
        self.p1_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(QLabel("X:"), 0, 1)
        pallet_inner.addWidget(self.p1_x_edit, 0, 2)
        pallet_inner.addWidget(QLabel("Y:"), 0, 3)
        pallet_inner.addWidget(self.p1_y_edit, 0, 4)
        pallet_inner.addWidget(QLabel("Z:"), 0, 5)
        pallet_inner.addWidget(self.p1_z_edit, 0, 6)
        pallet_inner.addWidget(QLabel("C:"), 0, 7)
        pallet_inner.addWidget(self.p1_c_edit, 0, 8)
        self.p1_record_btn = QPushButton("记录P1")
        self.p1_record_btn.clicked.connect(lambda: self.record_pallet_point(10, "P1"))
        pallet_inner.addWidget(self.p1_record_btn, 0, 9)

        pallet_inner.addWidget(QLabel("行上点 P2:"), 1, 0)
        self.p2_x_edit = QLineEdit("0")
        self.p2_y_edit = QLineEdit("0")
        self.p2_z_edit = QLineEdit("0")
        self.p2_c_edit = QLineEdit("0")
        self.p2_x_edit.setFixedWidth(60)
        self.p2_y_edit.setFixedWidth(60)
        self.p2_z_edit.setFixedWidth(60)
        self.p2_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(QLabel("X:"), 1, 1)
        pallet_inner.addWidget(self.p2_x_edit, 1, 2)
        pallet_inner.addWidget(QLabel("Y:"), 1, 3)
        pallet_inner.addWidget(self.p2_y_edit, 1, 4)
        pallet_inner.addWidget(QLabel("Z:"), 1, 5)
        pallet_inner.addWidget(self.p2_z_edit, 1, 6)
        pallet_inner.addWidget(QLabel("C:"), 1, 7)
        pallet_inner.addWidget(self.p2_c_edit, 1, 8)
        self.p2_record_btn = QPushButton("记录P2")
        self.p2_record_btn.clicked.connect(lambda: self.record_pallet_point(11, "P2"))
        pallet_inner.addWidget(self.p2_record_btn, 1, 9)

        pallet_inner.addWidget(QLabel("列上点 P3:"), 2, 0)
        self.p3_x_edit = QLineEdit("0")
        self.p3_y_edit = QLineEdit("0")
        self.p3_z_edit = QLineEdit("0")
        self.p3_c_edit = QLineEdit("0")
        self.p3_x_edit.setFixedWidth(60)
        self.p3_y_edit.setFixedWidth(60)
        self.p3_z_edit.setFixedWidth(60)
        self.p3_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(QLabel("X:"), 2, 1)
        pallet_inner.addWidget(self.p3_x_edit, 2, 2)
        pallet_inner.addWidget(QLabel("Y:"), 2, 3)
        pallet_inner.addWidget(self.p3_y_edit, 2, 4)
        pallet_inner.addWidget(QLabel("Z:"), 2, 5)
        pallet_inner.addWidget(self.p3_z_edit, 2, 6)
        pallet_inner.addWidget(QLabel("C:"), 2, 7)
        pallet_inner.addWidget(self.p3_c_edit, 2, 8)
        self.p3_record_btn = QPushButton("记录P3")
        self.p3_record_btn.clicked.connect(lambda: self.record_pallet_point(12, "P3"))
        pallet_inner.addWidget(self.p3_record_btn, 2, 9)

        self.apply_points_btn = QPushButton("应用点位修改")
        self.apply_points_btn.clicked.connect(self.apply_point_modifications)
        pallet_inner.addWidget(self.apply_points_btn, 3, 0, 1, 2)

        # 行列层参数
        pallet_inner.addWidget(QLabel("行数:"), 4, 0)
        self.row_edit = QLineEdit("3")
        self.row_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.row_edit, 4, 1)
        pallet_inner.addWidget(QLabel("列数:"), 4, 2)
        self.col_edit = QLineEdit("3")
        self.col_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.col_edit, 4, 3)
        pallet_inner.addWidget(QLabel("层数:"), 4, 4)
        self.layer_edit = QLineEdit("1")
        self.layer_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.layer_edit, 4, 5)
        pallet_inner.addWidget(QLabel("层高(mm):"), 4, 6)
        self.height_edit = QLineEdit("20")
        self.height_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.height_edit, 4, 7)
        pallet_inner.addWidget(QLabel("Z基准偏移:"), 4, 8)
        self.z_offset_edit = QLineEdit("0")
        self.z_offset_edit.setFixedWidth(50)
        pallet_inner.addWidget(self.z_offset_edit, 4, 9)

        # 间距模式
        pallet_inner.addWidget(QLabel("间距模式:"), 5, 0)
        self.spacing_mode_combo = QComboBox()
        self.spacing_mode_combo.addItems(["等分", "设定间距"])
        self.spacing_mode_combo.setCurrentIndex(0)
        self.spacing_mode_combo.currentIndexChanged.connect(self.on_spacing_mode_changed)
        pallet_inner.addWidget(self.spacing_mode_combo, 5, 1)

        self.row_dist_label = QLabel("行距(mm):")
        self.row_dist_edit = QLineEdit("50")
        self.row_dist_edit.setFixedWidth(60)
        self.col_dist_label = QLabel("列距(mm):")
        self.col_dist_edit = QLineEdit("50")
        self.col_dist_edit.setFixedWidth(60)
        pallet_inner.addWidget(self.row_dist_label, 5, 2)
        pallet_inner.addWidget(self.row_dist_edit, 5, 3)
        pallet_inner.addWidget(self.col_dist_label, 5, 4)
        pallet_inner.addWidget(self.col_dist_edit, 5, 5)
        self.row_dist_label.setVisible(False)
        self.row_dist_edit.setVisible(False)
        self.col_dist_label.setVisible(False)
        self.col_dist_edit.setVisible(False)

        # 趋近/离开偏移
        pallet_inner.addWidget(QLabel("趋近偏移Z:"), 6, 0)
        self.approach_offset_edit = QLineEdit("0")
        self.approach_offset_edit.setFixedWidth(60)
        pallet_inner.addWidget(self.approach_offset_edit, 6, 1)
        pallet_inner.addWidget(QLabel("离开偏移Z:"), 6, 2)
        self.depart_offset_edit = QLineEdit("0")
        self.depart_offset_edit.setFixedWidth(60)
        pallet_inner.addWidget(self.depart_offset_edit, 6, 3)

        # 安全点
        pallet_inner.addWidget(QLabel("安全点:"), 7, 0)
        self.safe_x_edit = QLineEdit("0")
        self.safe_y_edit = QLineEdit("0")
        self.safe_z_edit = QLineEdit("0")
        self.safe_c_edit = QLineEdit("0")
        self.safe_x_edit.setFixedWidth(60)
        self.safe_y_edit.setFixedWidth(60)
        self.safe_z_edit.setFixedWidth(60)
        self.safe_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(QLabel("X:"), 7, 1)
        pallet_inner.addWidget(self.safe_x_edit, 7, 2)
        pallet_inner.addWidget(QLabel("Y:"), 7, 3)
        pallet_inner.addWidget(self.safe_y_edit, 7, 4)
        pallet_inner.addWidget(QLabel("Z:"), 7, 5)
        pallet_inner.addWidget(self.safe_z_edit, 7, 6)
        pallet_inner.addWidget(QLabel("C:"), 7, 7)
        pallet_inner.addWidget(self.safe_c_edit, 7, 8)
        self.safe_record_btn = QPushButton("记录安全点")
        self.safe_record_btn.clicked.connect(self.record_safe_point)
        pallet_inner.addWidget(self.safe_record_btn, 7, 9)
        self.safe_move_btn = QPushButton("移动到安全点")
        self.safe_move_btn.clicked.connect(self.move_to_safe_point)
        pallet_inner.addWidget(self.safe_move_btn, 7, 10)

        # 拍照点
        pallet_inner.addWidget(QLabel("拍照点:"), 8, 0)
        self.photo_x_edit = QLineEdit("0")
        self.photo_y_edit = QLineEdit("0")
        self.photo_z_edit = QLineEdit("0")
        self.photo_c_edit = QLineEdit("0")
        self.photo_x_edit.setFixedWidth(60)
        self.photo_y_edit.setFixedWidth(60)
        self.photo_z_edit.setFixedWidth(60)
        self.photo_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(QLabel("X:"), 8, 1)
        pallet_inner.addWidget(self.photo_x_edit, 8, 2)
        pallet_inner.addWidget(QLabel("Y:"), 8, 3)
        pallet_inner.addWidget(self.photo_y_edit, 8, 4)
        pallet_inner.addWidget(QLabel("Z:"), 8, 5)
        pallet_inner.addWidget(self.photo_z_edit, 8, 6)
        pallet_inner.addWidget(QLabel("C:"), 8, 7)
        pallet_inner.addWidget(self.photo_c_edit, 8, 8)
        self.photo_record_btn = QPushButton("记录拍照点")
        self.photo_record_btn.clicked.connect(self.record_photo_point)
        pallet_inner.addWidget(self.photo_record_btn, 8, 9)
        self.photo_move_btn = QPushButton("移动到拍照点")
        self.photo_move_btn.clicked.connect(self.move_to_photo_point)
        pallet_inner.addWidget(self.photo_move_btn, 8, 10)

        # 运动类型和常规操作
        pallet_inner.addWidget(QLabel("运动类型:"), 9, 0)
        self.move_type_combo = QComboBox()
        self.move_type_combo.addItems(["MOVJ", "MOVL"])
        self.move_type_combo.setCurrentIndex(0)
        pallet_inner.addWidget(self.move_type_combo, 9, 1)

        self.generate_btn = QPushButton("生成码垛点")
        self.generate_btn.clicked.connect(self.on_generate_pallet_points)
        pallet_inner.addWidget(self.generate_btn, 9, 2)
        self.move_selected_btn = QPushButton("移动到选中点")
        self.move_selected_btn.clicked.connect(self.on_move_selected)
        pallet_inner.addWidget(self.move_selected_btn, 9, 3)
        self.run_all_btn = QPushButton("连续运行所有点")
        self.run_all_btn.clicked.connect(self.on_run_all)
        pallet_inner.addWidget(self.run_all_btn, 9, 4)
        self.stop_run_btn = QPushButton("停止运行")
        self.stop_run_btn.clicked.connect(self.on_stop_run)
        self.stop_run_btn.setEnabled(False)
        pallet_inner.addWidget(self.stop_run_btn, 9, 5)
        self.reset_pallet_btn = QPushButton("重置码垛")
        self.reset_pallet_btn.clicked.connect(self.on_reset_pallet)
        pallet_inner.addWidget(self.reset_pallet_btn, 9, 6)

        # 视觉码垛
        pallet_inner.addWidget(QLabel("视觉码垛:"), 10, 0)
        self.vision_pallet_btn = QPushButton("开始视觉码垛")
        self.vision_pallet_btn.clicked.connect(self.on_vision_pallet)
        pallet_inner.addWidget(self.vision_pallet_btn, 10, 1)
        self.stop_vision_pallet_btn = QPushButton("停止视觉码垛")
        self.stop_vision_pallet_btn.clicked.connect(self.on_stop_vision_pallet)
        self.stop_vision_pallet_btn.setEnabled(False)
        pallet_inner.addWidget(self.stop_vision_pallet_btn, 10, 2)
        pallet_inner.addWidget(QLabel("抓取C值:"), 10, 3)
        self.grab_c_edit = QLineEdit("0")
        self.grab_c_edit.setFixedWidth(60)
        pallet_inner.addWidget(self.grab_c_edit, 10, 4)
        self.use_manual_c_check = QCheckBox("用手动C覆盖视觉C")
        pallet_inner.addWidget(self.use_manual_c_check, 10, 5)

        # 点列表
        pallet_inner.addWidget(QLabel("码垛点列表 (点击选中):"), 11, 0, 1, 6)
        self.point_table = QTableWidget()
        self.point_table.setColumnCount(5)
        self.point_table.setHorizontalHeaderLabels(["序号", "X", "Y", "Z", "C"])
        self.point_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.point_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.point_table.setSelectionMode(QTableWidget.SingleSelection)
        self.point_table.clicked.connect(self.on_table_point_clicked)
        pallet_inner.addWidget(self.point_table, 12, 0, 1, 6)

        # 进度
        pallet_inner.addWidget(QLabel("进度:"), 13, 0)
        self.progress_label = QLabel("0 / 0")
        pallet_inner.addWidget(self.progress_label, 13, 1)
        pallet_inner.addWidget(QLabel("当前目标:"), 13, 2)
        self.current_target_label = QLabel("--")
        pallet_inner.addWidget(self.current_target_label, 13, 3, 1, 3)

        pallet_inner.addWidget(QLabel("提示: 记录P1/P2/P3→生成点→记录拍照点→开始视觉码垛"), 14, 0, 1, 6)

        pallet_group.setLayout(pallet_inner)
        pallet_layout.addWidget(pallet_group)
        pallet_layout.addStretch()
        self.tab_widget.addTab(pallet_tab, "手动码垛")

        left_layout.addStretch()

        # 日志面板
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

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_widget)
        splitter.addWidget(log_group)
        splitter.setSizes([int(self.width() * 0.7), int(self.width() * 0.3)])
        main_layout.addWidget(splitter)

        self._set_robot_controls_enabled(False)
        self._set_vm_controls_enabled(False)
        self._set_calib_controls_enabled(False)

    # ===== 辅助 =====
    def _set_robot_controls_enabled(self, enabled):
        for w in [self.movj_btn, self.emergency_btn,
                  self.speed_slider, self.mode_auto_btn, self.mode_autocmd_btn,
                  self.mode_manual_btn, self.servo_on_btn, self.servo_off_btn,
                  self.reset_alarm_btn, self.switch_uf_btn, self.switch_tf_btn,
                  self.p1_record_btn, self.p2_record_btn, self.p3_record_btn,
                  self.safe_record_btn, self.safe_move_btn,
                  self.photo_record_btn, self.photo_move_btn,
                  self.apply_points_btn,
                  self.generate_btn, self.move_selected_btn, self.run_all_btn, self.stop_run_btn,
                  self.reset_pallet_btn, self.vision_pallet_btn, self.stop_vision_pallet_btn]:
            w.setEnabled(enabled)

    def _set_vm_controls_enabled(self, enabled):
        self.vm_disconnect_btn.setEnabled(enabled)
        self.vm_trigger_btn.setEnabled(enabled)

    def _set_calib_controls_enabled(self, enabled):
        for w in [self.record_base_btn, self.apply_config_btn,
                  self.start_calib_btn, self.verify_btn, self.diagnose_btn,
                  self.save_calib_btn, self.load_calib_btn]:
            w.setEnabled(enabled)

    def on_spacing_mode_changed(self, index):
        if index == 0:
            self.row_dist_label.setVisible(False)
            self.row_dist_edit.setVisible(False)
            self.col_dist_label.setVisible(False)
            self.col_dist_edit.setVisible(False)
        else:
            self.row_dist_label.setVisible(True)
            self.row_dist_edit.setVisible(True)
            self.col_dist_label.setVisible(True)
            self.col_dist_edit.setVisible(True)

    # ===== 机器人连接 =====
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
            self.is_robot_connected = False
            self.state_timer.stop()
            self.robot_status_label.setText("🔴 连接失败")
            self.robot_connect_btn.setEnabled(True)
            self.robot_disconnect_btn.setEnabled(False)
            self._set_robot_controls_enabled(False)
            self._set_calib_controls_enabled(False)

    def on_robot_connection_lost(self):
        self.append_log("[重连] 检测到机器人断线，将在 2 秒后自动重连...")
        QTimer.singleShot(2000, self._reconnect_robot)

    def _reconnect_robot(self):
        if self.is_robot_connected:
            return
        self.append_log("[重连] 正在重新连接机器人...")
        self.robot_worker.connect_robot()

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

    def on_emergency_stop(self):
        self.stop_requested = True
        self.run_all_flag = False
        self.stop_run_btn.setEnabled(False)
        self.run_all_btn.setEnabled(True)
        self.vision_pallet_stop = True
        self.vision_pallet_running = False
        self.stop_vision_pallet_btn.setEnabled(False)
        self.vision_pallet_btn.setEnabled(True)
        self.robot_worker.abort_immediately()
        self.append_log("[急停] 已触发：机器人停止，运动命令已锁定，请复位报警后继续")

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

    # ===== Vision Master =====
    def on_vm_connect(self):
        ip = self.vm_ip_edit.text().strip()
        try:
            port = int(self.vm_port_edit.text().strip())
        except ValueError:
            self.append_log("[错误] 端口必须为数字")
            return
        trigger = self.vm_trigger_edit.text().strip()
        try:
            timeout = float(self.vm_timeout_edit.text().strip())
        except ValueError:
            timeout = 3.0
        self.append_log(f"正在连接 Vision Master {ip}:{port} (超时 {timeout}s)")
        self.vm_thread.set_params(ip, port, trigger, timeout)
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
            self.append_log("[VM] ✅ ATTR 包含 0,1,2,3")
        else:
            self.append_log("[VM] ⚠️ ATTR 缺少部分值")
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

    # ===== 标定 =====
    def get_current_pose(self):
        if self.robot_worker.handle is None:
            return None
        try:
            with X5_LOCK:
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
                self.append_log("[预检查] 触发拍照超时")
                self.vm_thread.data_received.disconnect(on_data)
                return
            self.vm_thread.data_received.disconnect(on_data)

            if not received_points:
                self.append_log("[预检查] 未收到任何视觉数据")
                return
            attrs = set([p['attr'] for p in received_points])
            if not (0 in attrs and 1 in attrs and 2 in attrs and 3 in attrs):
                self.append_log(f"[预检查] ATTR不完整: {sorted(attrs)}")
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
            params = self._collect_calib_params(vision_idx)
            config = self._build_config(params)
            calib_process = self._build_calib_process(params)
            self.last_calib_params[vision_idx] = params
            self.robot_worker.apply_vision_config(vision_idx, config, calib_process)
            self.calib_result_label.setText("标定状态: ✅ 配置已写入")
            self.calib_result_label.setStyleSheet("background-color: #d4edda; padding: 5px;")
            self.append_log("✅ Vision配置和标定参数写入成功")
        except Exception as e:
            self.append_log(f"[配置] 异常: {str(e)}")
            traceback.print_exc()
            self.calib_result_label.setText("标定状态: ❌ 配置异常")
            self.calib_result_label.setStyleSheet("background-color: #f8d7da; padding: 5px;")

    # ===== 标定结果保存 / 导入 =====
    def _point_to_dict(self, point):
        if point is None:
            return None
        p = point.pose
        cfg = list(point.cfg) if hasattr(point, 'cfg') else [0, 0, 0, 1]
        return {
            'x': p.x, 'y': p.y, 'z': p.z,
            'a': p.a, 'b': p.b, 'c': p.c,
            'uf': point.uf, 'tf': point.tf, 'cfg': cfg,
        }

    def _point_from_dict(self, d):
        if not d:
            return None
        pose = x5.Pose(d['x'], d['y'], d['z'], d['a'], d['b'], d['c'], 0, 0, 0)
        return x5.Point(pose=pose, uf=d.get('uf', 0), tf=d.get('tf', 0),
                        cfg=tuple(d.get('cfg', [0, 0, 0, 1])))

    def _collect_calib_params(self, vision_idx):
        """从界面采集本次标定所需的全部参数（可 JSON 序列化）"""
        try:
            port = int(self.vm_port_edit.text().strip())
        except ValueError:
            port = 4004
        try:
            timeout = float(self.vm_timeout_edit.text().strip())
        except ValueError:
            timeout = 3.0
        return {
            'vision_idx': vision_idx,
            'ip': self.vm_ip_edit.text().strip(),
            'port': port,
            'protocol': 0 if self.vm_protocol_combo.currentIndex() == 0 else 1,
            'trigger_type': 0 if self.vm_trigger_type_combo.currentIndex() == 0 else 1,
            'trigger_cmd': self.vm_trigger_edit.text().strip(),
            'data_format': int(self.vm_format_combo.currentText()),
            'invert_c': self.vm_c_invert_check.isChecked(),
            'uf_index': self.vm_uf_combo.currentIndex(),
            'tf_index': self.vm_tf_combo.currentIndex(),
            'mount_type': 2,
            'calib_object': 0 if self.vm_calib_object_combo.currentIndex() == 0 else 1,
            'calib_type': 0 if self.vm_calib_type_combo.currentIndex() == 0 else 1,
            'point_count': self.vm_calib_points_combo.currentIndex(),
            'timeout': timeout,
            'pixel_u': int(self.pixel_u_edit.text()),
            'pixel_v': int(self.pixel_v_edit.text()),
            'mark_distance': int(self.mark_distance_edit.text()),
            'step_size': int(self.step_size_edit.text()),
            'base_point': self._point_to_dict(self.base_point),
        }

    def _build_config(self, params):
        return x5v.Vision(
            ip=str(params['ip']).encode(),
            port=int(params['port']),
            communication_protocol=int(params['protocol']),
            auto_start=False,
            data_format=int(params['data_format']),
            invert_c_value=bool(params['invert_c']),
            camera_mount_type=int(params['mount_type']),
            uf_index=int(params['uf_index']),
            calibration_object=int(params['calib_object']),
            trigger_type=int(params['trigger_type']),
            trigger_do=255,
            trigger_command=str(params['trigger_cmd']).encode()
        )

    def _build_calib_process(self, params):
        calib_process = x5v.VisionCalibProcess()
        calib_process.calibration_type = int(params['calib_type'])
        calib_process.point_count = int(params['point_count'])
        calib_process.is_workplane_parallel = True
        calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
            tf_index=int(params['tf_index']),
            pixel_shift=[int(params['pixel_u']), int(params['pixel_v'])],
            error=0.0,
            base_point=self._point_from_dict(params['base_point']),
            mark_distance=int(params['mark_distance']),
            step_size=int(params['step_size'])
        )
        return calib_process

    def _apply_calib_params_to_ui(self, params):
        """把导入的标定参数回填到界面控件，方便用户查看/再次保存"""
        self.vm_ip_edit.setText(str(params.get('ip', '')))
        self.vm_port_edit.setText(str(params.get('port', 4004)))
        self.vm_protocol_combo.setCurrentIndex(0 if params.get('protocol') == 0 else 1)
        self.vm_trigger_type_combo.setCurrentIndex(0 if params.get('trigger_type') == 0 else 1)
        self.vm_trigger_edit.setText(str(params.get('trigger_cmd', 'trigger')))
        self.vm_format_combo.setCurrentIndex(self.vm_format_combo.findText(str(params.get('data_format', 1))))
        self.vm_c_invert_check.setChecked(bool(params.get('invert_c', False)))
        self.vm_uf_combo.setCurrentIndex(int(params.get('uf_index', 15)))
        self.vm_tf_combo.setCurrentIndex(int(params.get('tf_index', 0)))
        self.vm_calib_object_combo.setCurrentIndex(0 if params.get('calib_object') == 0 else 1)
        self.vm_calib_type_combo.setCurrentIndex(0 if params.get('calib_type') == 0 else 1)
        self.vm_calib_points_combo.setCurrentIndex(int(params.get('point_count', 0)))
        self.vm_timeout_edit.setText(str(params.get('timeout', 3.0)))
        self.pixel_u_edit.setText(str(params.get('pixel_u', 2592)))
        self.pixel_v_edit.setText(str(params.get('pixel_v', 2048)))
        self.mark_distance_edit.setText(str(params.get('mark_distance', 10)))
        self.step_size_edit.setText(str(params.get('step_size', 10)))
        bp = params.get('base_point')
        if bp:
            self.base_point = self._point_from_dict(bp)
            self.base_point_label.setText(
                f"基准点: X={bp['x']:.2f} Y={bp['y']:.2f} Z={bp['z']:.2f} "
                f"A={bp['a']:.2f} B={bp['b']:.2f} C={bp['c']:.2f} (UF{bp['uf']}, TF{bp['tf']})"
            )

    @property
    def _calib_file_path(self):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_calib.json")

    def on_save_calib(self):
        """保存当前视觉工艺的标定结果到本地文件"""
        vision_idx = int(self.vision_index_combo.currentText().split()[-1])
        if not self.calibration_status.get(vision_idx, False) or not self.calib_data.get(vision_idx):
            QMessageBox.warning(self, "提示", f"视觉工艺 {vision_idx} 尚未完成标定，请先标定")
            return
        try:
            params = self._collect_calib_params(vision_idx)
            params['transformation'] = [float(x) for x in self.calib_data[vision_idx]]

            data = {}
            if os.path.exists(self._calib_file_path):
                try:
                    with open(self._calib_file_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                except Exception:
                    data = {}
            data[str(vision_idx)] = params
            with open(self._calib_file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self.append_log(f"[标定] ✅ 视觉工艺 {vision_idx} 标定结果已保存: {self._calib_file_path}")
            QMessageBox.information(self, "提示", f"视觉工艺 {vision_idx} 标定结果已保存")
        except Exception as e:
            self.append_log(f"[标定] 保存失败: {e}")
            QMessageBox.warning(self, "提示", f"保存标定失败: {e}")

    def on_load_calib(self):
        """从本地文件导入上次保存的视觉工艺标定结果并写入机器人"""
        if not self.is_robot_connected:
            QMessageBox.warning(self, "提示", "请先连接机器人")
            return
        if not os.path.exists(self._calib_file_path):
            QMessageBox.warning(self, "提示", f"未找到标定文件: {self._calib_file_path}")
            return
        try:
            with open(self._calib_file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception as e:
            self.append_log(f"[标定] 读取标定文件失败: {e}")
            QMessageBox.warning(self, "提示", f"读取标定文件失败: {e}")
            return

        vision_idx = int(self.vision_index_combo.currentText().split()[-1])
        key = str(vision_idx)
        if key not in data:
            QMessageBox.warning(self, "提示", f"标定文件中没有视觉工艺 {vision_idx} 的标定数据")
            return
        params = data[key]
        transformation = params.get('transformation')
        if not transformation or len(transformation) != 12:
            QMessageBox.warning(self, "提示", "标定文件中变换矩阵缺失或长度不正确")
            return

        try:
            # 回填界面
            self._apply_calib_params_to_ui(params)
            # 写入配置 + 标定工艺
            config = self._build_config(params)
            calib_process = self._build_calib_process(params)
            self.robot_worker.apply_vision_config(vision_idx, config, calib_process)
            # 写入变换矩阵
            handle = self.robot_worker.handle
            if handle is not None:
                with X5_LOCK:
                    x5v.vision_set_calib_par(handle, vision_idx, [float(x) for x in transformation])
            self.calib_data[vision_idx] = [float(x) for x in transformation]
            self.calibration_status[vision_idx] = True
            self.last_calib_params[vision_idx] = params
            self.calib_result_label.setText(f"标定状态: ✅ 已导入 vision {vision_idx} 标定结果")
            self.calib_result_label.setStyleSheet("background-color: #d4edda; padding: 5px;")
            self.append_log(f"[标定] ✅ 视觉工艺 {vision_idx} 标定结果已导入并写入机器人")
        except Exception as e:
            self.append_log(f"[标定] 导入失败: {e}")
            traceback.print_exc()
            QMessageBox.warning(self, "提示", f"导入标定失败: {e}")

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
            self.calibration_status[vision_idx] = True
            self.calib_data[vision_idx] = [float(x) for x in transformation]  # 缓存，供“保存标定”使用
            self.append_log(f"[标定] 视觉工艺 {vision_idx} 已标定")
            try:
                with X5_LOCK:
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

    # ============================================================
    # ===== 码垛手动控制 =====
    # ============================================================
    def record_pallet_point(self, pr_index, name):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        uf = pose[6]
        tf = pose[7]
        cfg = pose[8] if len(pose) > 8 else (0, 0, 0, 1)
        p = pose[:6]
        pose_obj = x5.Pose(p[0], p[1], p[2], p[3], p[4], p[5], 0, 0, 0)
        point_obj = x5.Point(pose=pose_obj, uf=uf, tf=tf, cfg=cfg)

        self.robot_worker.set_pr_value(pr_index, point_obj)

        if name == "P1":
            self.p1_x_edit.setText(f"{p[0]:.2f}")
            self.p1_y_edit.setText(f"{p[1]:.2f}")
            self.p1_z_edit.setText(f"{p[2]:.2f}")
            self.p1_c_edit.setText(f"{p[5]:.2f}")
            self.p1_point = point_obj
        elif name == "P2":
            self.p2_x_edit.setText(f"{p[0]:.2f}")
            self.p2_y_edit.setText(f"{p[1]:.2f}")
            self.p2_z_edit.setText(f"{p[2]:.2f}")
            self.p2_c_edit.setText(f"{p[5]:.2f}")
            self.p2_point = point_obj
        elif name == "P3":
            self.p3_x_edit.setText(f"{p[0]:.2f}")
            self.p3_y_edit.setText(f"{p[1]:.2f}")
            self.p3_z_edit.setText(f"{p[2]:.2f}")
            self.p3_c_edit.setText(f"{p[5]:.2f}")
            self.p3_point = point_obj

        self.pallet_uf = uf
        self.pallet_tf = tf
        self.pallet_cfg = cfg
        self.append_log(f"[码垛] {name} (PR{pr_index}) 已记录并更新编辑框")

    def apply_point_modifications(self):
        try:
            p1_x = float(self.p1_x_edit.text())
            p1_y = float(self.p1_y_edit.text())
            p1_z = float(self.p1_z_edit.text())
            p1_c = float(self.p1_c_edit.text())
            p2_x = float(self.p2_x_edit.text())
            p2_y = float(self.p2_y_edit.text())
            p2_z = float(self.p2_z_edit.text())
            p2_c = float(self.p2_c_edit.text())
            p3_x = float(self.p3_x_edit.text())
            p3_y = float(self.p3_y_edit.text())
            p3_z = float(self.p3_z_edit.text())
            p3_c = float(self.p3_c_edit.text())
        except ValueError:
            self.append_log("[错误] 坐标值必须为数字")
            return

        uf = self.pallet_uf
        tf = self.pallet_tf
        cfg = self.pallet_cfg

        p1_pose = x5.Pose(p1_x, p1_y, p1_z, 0, 0, p1_c, 0, 0, 0)
        p1_point = x5.Point(pose=p1_pose, uf=uf, tf=tf, cfg=cfg)
        self.p1_point = p1_point
        self.robot_worker.set_pr_value(10, p1_point)

        p2_pose = x5.Pose(p2_x, p2_y, p2_z, 0, 0, p2_c, 0, 0, 0)
        p2_point = x5.Point(pose=p2_pose, uf=uf, tf=tf, cfg=cfg)
        self.p2_point = p2_point
        self.robot_worker.set_pr_value(11, p2_point)

        p3_pose = x5.Pose(p3_x, p3_y, p3_z, 0, 0, p3_c, 0, 0, 0)
        p3_point = x5.Point(pose=p3_pose, uf=uf, tf=tf, cfg=cfg)
        self.p3_point = p3_point
        self.robot_worker.set_pr_value(12, p3_point)

        self.append_log("[码垛] 点位修改已应用，PR10/PR11/PR12已更新")

    def record_safe_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        uf = pose[6]
        tf = pose[7]
        cfg = pose[8] if len(pose) > 8 else (0, 0, 0, 1)
        p = pose[:6]
        pose_obj = x5.Pose(p[0], p[1], p[2], p[3], p[4], p[5], 0, 0, 0)
        point_obj = x5.Point(pose=pose_obj, uf=uf, tf=tf, cfg=cfg)
        self.robot_worker.set_pr_value(20, point_obj)
        self.safe_point = point_obj
        self.safe_x_edit.setText(f"{p[0]:.2f}")
        self.safe_y_edit.setText(f"{p[1]:.2f}")
        self.safe_z_edit.setText(f"{p[2]:.2f}")
        self.safe_c_edit.setText(f"{p[5]:.2f}")
        self.append_log("[码垛] 安全点已记录到 PR20")

    def move_to_safe_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.safe_point is None:
            self.append_log("[错误] 请先记录安全点")
            return
        move_type = self.move_type_combo.currentText()
        if move_type == "MOVJ":
            self.robot_worker.movj_to_pose(self.safe_point.pose, self.safe_point.uf, self.safe_point.tf, self.safe_point.cfg)
        else:
            self.robot_worker.movl_to_pose(self.safe_point.pose, self.safe_point.uf, self.safe_point.tf, self.safe_point.cfg)

    def record_photo_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        pose = self.get_current_pose()
        if pose is None:
            self.append_log("[错误] 获取当前位置失败")
            return
        uf = pose[6]
        tf = pose[7]
        cfg = pose[8] if len(pose) > 8 else (0, 0, 0, 1)
        p = pose[:6]
        pose_obj = x5.Pose(p[0], p[1], p[2], p[3], p[4], p[5], 0, 0, 0)
        point_obj = x5.Point(pose=pose_obj, uf=uf, tf=tf, cfg=cfg)
        self.robot_worker.set_pr_value(14, point_obj)
        self.photo_point = point_obj
        self.photo_x_edit.setText(f"{p[0]:.2f}")
        self.photo_y_edit.setText(f"{p[1]:.2f}")
        self.photo_z_edit.setText(f"{p[2]:.2f}")
        self.photo_c_edit.setText(f"{p[5]:.2f}")
        self.append_log("[码垛] 拍照点已记录到 PR14")

    def move_to_photo_point(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.photo_point is None:
            self.append_log("[错误] 请先记录拍照点")
            return
        move_type = self.move_type_combo.currentText()
        if move_type == "MOVJ":
            self.robot_worker.movj_to_pose(self.photo_point.pose, self.photo_point.uf, self.photo_point.tf, self.photo_point.cfg)
        else:
            self.robot_worker.movl_to_pose(self.photo_point.pose, self.photo_point.uf, self.photo_point.tf, self.photo_point.cfg)

    def on_generate_pallet_points(self):
        if not self.is_robot_connected:
            self.append_log("[错误] 请先连接机器人")
            return
        if self.p1_point is None or self.p2_point is None or self.p3_point is None:
            QMessageBox.warning(self, "提示", "请先记录P1、P2、P3点或手动输入并应用")
            return

        try:
            row = int(self.row_edit.text())
            col = int(self.col_edit.text())
            layer = int(self.layer_edit.text())
            height = float(self.height_edit.text())
            z_offset = float(self.z_offset_edit.text())
            approach_offset = float(self.approach_offset_edit.text())
            depart_offset = float(self.depart_offset_edit.text())
            spacing_mode = self.spacing_mode_combo.currentIndex()
            if spacing_mode == 0:
                row_dist = None
                col_dist = None
            else:
                row_dist = float(self.row_dist_edit.text())
                col_dist = float(self.col_dist_edit.text())
        except ValueError as e:
            self.append_log(f"[错误] 参数输入无效: {e}")
            return

        p1 = self.p1_point.pose
        p2 = self.p2_point.pose
        p3 = self.p3_point.pose

        row_vec = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        col_vec = (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)

        if spacing_mode == 0:
            if row > 1:
                row_step = (row_vec[0] / (row - 1), row_vec[1] / (row - 1), row_vec[2] / (row - 1))
            else:
                row_step = (0, 0, 0)
            if col > 1:
                col_step = (col_vec[0] / (col - 1), col_vec[1] / (col - 1), col_vec[2] / (col - 1))
            else:
                col_step = (0, 0, 0)
        else:
            row_len = math.sqrt(row_vec[0]**2 + row_vec[1]**2 + row_vec[2]**2)
            col_len = math.sqrt(col_vec[0]**2 + col_vec[1]**2 + col_vec[2]**2)
            if row_len < 1e-6 or col_len < 1e-6:
                self.append_log("[错误] P2或P3与P1重合")
                return
            row_unit = (row_vec[0] / row_len, row_vec[1] / row_len, row_vec[2] / row_len)
            col_unit = (col_vec[0] / col_len, col_vec[1] / col_len, col_vec[2] / col_len)
            row_step = (row_unit[0] * row_dist, row_unit[1] * row_dist, row_unit[2] * row_dist)
            col_step = (col_unit[0] * col_dist, col_unit[1] * col_dist, col_unit[2] * col_dist)

        points = []
        for k in range(layer):
            z_layer_offset = k * height
            for i in range(row):
                for j in range(col):
                    x = p1.x + i * row_step[0] + j * col_step[0]
                    y = p1.y + i * row_step[1] + j * col_step[1]
                    z_base = p1.z + i * row_step[2] + j * col_step[2] + z_layer_offset
                    z = z_base + z_offset
                    pose = x5.Pose(x, y, z, p1.a, p1.b, p1.c, 0, 0, 0)
                    points.append(pose)

        if not points:
            self.append_log("[错误] 生成点数为0，请检查参数")
            return

        self.pallet_points = points
        self.total_steps = len(points)
        self.current_step = 0
        self.progress_label.setText(f"0 / {self.total_steps}")
        self.current_target_label.setText("--")
        self.point_table.setRowCount(self.total_steps)
        for idx, pose in enumerate(points):
            self.point_table.setItem(idx, 0, QTableWidgetItem(str(idx+1)))
            self.point_table.setItem(idx, 1, QTableWidgetItem(f"{pose.x:.2f}"))
            self.point_table.setItem(idx, 2, QTableWidgetItem(f"{pose.y:.2f}"))
            self.point_table.setItem(idx, 3, QTableWidgetItem(f"{pose.z:.2f}"))
            self.point_table.setItem(idx, 4, QTableWidgetItem(f"{pose.c:.2f}"))
        self.point_table.resizeColumnsToContents()
        self.append_log(f"[码垛] 成功生成 {self.total_steps} 个目标点（Z基准偏移 {z_offset:.1f} mm）")

    def on_table_point_clicked(self, index):
        self.selected_row = index.row()
        self.append_log(f"[码垛] 选中第 {self.selected_row+1} 个点")

    def on_move_selected(self):
        if self.selected_row < 0 or self.selected_row >= len(self.pallet_points):
            self.append_log("[错误] 请先选择一个点")
            return
        self._move_to_point(self.selected_row)

    def _move_to_point(self, idx, move_type=None, approach_offset=None, depart_offset=None, points=None):
        if points is None:
            points = self.pallet_points
        if idx < 0 or idx >= len(points):
            return

        # 从 GUI 线程直接调用（单点移动）时读取控件；工作线程传入捕获值
        if move_type is None:
            move_type = self.move_type_combo.currentText()
        if approach_offset is None:
            try:
                approach_offset = float(self.approach_offset_edit.text())
            except ValueError:
                approach_offset = 0.0
        if depart_offset is None:
            try:
                depart_offset = float(self.depart_offset_edit.text())
            except ValueError:
                depart_offset = 0.0

        handle = self.robot_worker.handle
        if handle is None:
            self.append_log("[错误] 机器人未连接")
            return
        try:
            with X5_LOCK:
                state = x5.get_system_state(handle)
                alarm = state.alarm
                enable = state.enable
                mode = state.mode
            if alarm:
                self.append_log(f"[错误] 机器人有报警（代码{alarm}），请先复位报警")
                return
            if not enable:
                self.append_log("[错误] 机器人未上使能，请先上使能")
                return
            if mode not in (2, 100):
                self.append_log(f"[提示] 当前模式{mode}，正在切换为自动命令模式...")
                with X5_LOCK:
                    x5.set_system_mode(handle, 100)
                time.sleep(0.3)
                with X5_LOCK:
                    state = x5.get_system_state(handle)
                if state.mode != 100:
                    self.append_log("[错误] 切换自动命令模式失败，请手动切换")
                    return
        except Exception as e:
            self.append_log(f"[错误] 检查机器人状态异常: {e}")
            return

        target_pose = points[idx]
        uf = self.pallet_uf
        tf = self.pallet_tf
        cfg = self.pallet_cfg

        approach_z = target_pose.z + approach_offset
        approach_pose = x5.Pose(target_pose.x, target_pose.y, approach_z,
                                target_pose.a, target_pose.b, target_pose.c, 0, 0, 0)
        depart_z = target_pose.z + depart_offset
        depart_pose = x5.Pose(target_pose.x, target_pose.y, depart_z,
                              target_pose.a, target_pose.b, target_pose.c, 0, 0, 0)

        self.append_log(f"[码垛] 移动到第 {idx+1} 个点，运动类型: {move_type}")

        if move_type == "MOVJ":
            self.robot_worker.movj_to_pose(approach_pose, uf, tf, cfg)
            self.robot_worker.movj_to_pose(target_pose, uf, tf, cfg)
            self.robot_worker.movj_to_pose(depart_pose, uf, tf, cfg)
        else:
            self.robot_worker.movl_to_pose(approach_pose, uf, tf, cfg)
            self.robot_worker.movl_to_pose(target_pose, uf, tf, cfg)
            self.robot_worker.movl_to_pose(depart_pose, uf, tf, cfg)

        if not self.run_all_flag:
            self.current_step = idx + 1
            self.progress_label.setText(f"{self.current_step} / {self.total_steps}")
            self.current_target_label.setText(f"X={target_pose.x:.1f} Y={target_pose.y:.1f} Z={target_pose.z:.1f}")

    def on_run_all(self):
        if not self.pallet_points:
            self.append_log("[错误] 请先生成码垛点")
            return
        if self.run_all_flag:
            return
        if self.robot_worker._stop_flag:
            self.append_log("[错误] 当前处于急停锁定状态，请点击“复位报警”解锁")
            QMessageBox.warning(self, "提示", "急停锁定未解除，请先点击“复位报警”解锁。")
            return

        handle = self.robot_worker.handle
        if handle is None:
            self.append_log("[错误] 机器人未连接")
            QMessageBox.warning(self, "提示", "机器人未连接")
            return
        try:
            with X5_LOCK:
                state = x5.get_system_state(handle)
                alarm = state.alarm
                enable = state.enable
                mode = state.mode
            if alarm:
                self.append_log("[错误] 机器人有报警，请先复位报警")
                QMessageBox.warning(self, "提示", "机器人有报警，请先点击“复位报警”")
                return
            if not enable:
                self.append_log("[提示] 机器人未上使能，正在上使能...")
                with X5_LOCK:
                    x5.enable_servo(handle, True)
                time.sleep(0.5)
                with X5_LOCK:
                    state = x5.get_system_state(handle)
                    enable = state.enable
                if not enable:
                    self.append_log("[错误] 上使能失败")
                    QMessageBox.warning(self, "提示", "上使能失败，请手动上使能")
                    return
            if mode not in (2, 100):
                self.append_log(f"[提示] 当前模式不是自动/自动命令模式 (当前={mode})，正在切换到自动命令模式...")
                with X5_LOCK:
                    x5.set_system_mode(handle, 100)
                for _ in range(20):
                    time.sleep(0.1)
                    with X5_LOCK:
                        state = x5.get_system_state(handle)
                    mode = state.mode
                    if mode == 100:
                        break
                if mode != 100:
                    self.append_log("[错误] 切换模式失败")
                    QMessageBox.warning(self, "提示", "切换模式失败，请手动切换到自动模式或自动命令模式")
                    return
                self.append_log("[提示] 已切换到自动命令模式")
            self.append_log("[提示] 机器人状态已准备就绪")
        except Exception as e:
            self.append_log(f"[错误] 准备机器人状态时出错: {e}")
            QMessageBox.warning(self, "提示", f"准备机器人状态失败: {e}")
            return

        # 在线程启动前捕获参数，工作线程不再直接读取 Qt 控件
        run_points = list(self.pallet_points)
        run_total = len(run_points)
        run_move_type = self.move_type_combo.currentText()
        try:
            run_approach = float(self.approach_offset_edit.text())
        except ValueError:
            run_approach = 0.0
        try:
            run_depart = float(self.depart_offset_edit.text())
        except ValueError:
            run_depart = 0.0

        self.run_all_flag = True
        self.stop_requested = False
        self.stop_run_btn.setEnabled(True)
        self.run_all_btn.setEnabled(False)
        self.append_log("[码垛] 开始连续运行所有点")
        import threading
        def run_thread():
            for i in range(run_total):
                if self.stop_requested:
                    break
                if not self.run_all_flag:
                    break
                QMetaObject.invokeMethod(self, "update_progress", Qt.QueuedConnection,
                                         Q_ARG(int, i+1), Q_ARG(int, run_total),
                                         Q_ARG(float, run_points[i].x),
                                         Q_ARG(float, run_points[i].y),
                                         Q_ARG(float, run_points[i].z))
                self._move_to_point(i, run_move_type, run_approach, run_depart, run_points)
                time.sleep(0.5)
            self.run_all_flag = False
            self.stop_requested = False
            QMetaObject.invokeMethod(self, "on_run_finished", Qt.QueuedConnection)
        threading.Thread(target=run_thread, daemon=True).start()

    @pyqtSlot(int, int, float, float, float)
    def update_progress(self, current, total, x, y, z):
        self.progress_label.setText(f"{current} / {total}")
        self.current_target_label.setText(f"X={x:.1f} Y={y:.1f} Z={z:.1f}")

    @pyqtSlot()
    def on_run_finished(self):
        self.stop_run_btn.setEnabled(False)
        self.run_all_btn.setEnabled(True)
        if not self.stop_requested:
            self.append_log("[码垛] 连续运行结束")
        else:
            self.append_log("[码垛] 连续运行已停止")

    def on_stop_run(self):
        self.stop_requested = True
        self.run_all_flag = False
        self.stop_run_btn.setEnabled(False)
        self.run_all_btn.setEnabled(True)
        self.robot_worker.abort_immediately(lock_motion=False)
        self.append_log("[码垛] 已请求停止运行")

    def on_reset_pallet(self):
        self.current_step = 0
        self.pallet_points = []
        self.total_steps = 0
        self.point_table.setRowCount(0)
        self.progress_label.setText("0 / 0")
        self.current_target_label.setText("--")
        self.selected_row = -1
        self.run_all_flag = False
        self.stop_requested = False
        self.stop_run_btn.setEnabled(False)
        self.run_all_btn.setEnabled(True)
        self.append_log("[码垛] 已重置")

    # ============================================================
    # ===== 视觉码垛 =====
    # ============================================================
    def on_vision_pallet(self):
        """开始视觉码垛流程（同步触发，一次拍照获取多个工件）"""
        if not self.is_robot_connected:
            self.append_log("[错误] 机器人未连接")
            QMessageBox.warning(self, "提示", "请先连接机器人")
            return
        if not self.is_vm_connected:
            self.append_log("[错误] Vision Master 未连接")
            QMessageBox.warning(self, "提示", "请先连接 Vision Master")
            return
        if not self.pallet_points:
            self.append_log("[错误] 未生成码垛点")
            QMessageBox.warning(self, "提示", "请先生成码垛点")
            return
        if self.photo_point is None:
            self.append_log("[错误] 未记录拍照点")
            QMessageBox.warning(self, "提示", "请先记录拍照点")
            return
        if self.robot_worker._stop_flag:
            self.append_log("[错误] 急停锁定，请复位报警")
            QMessageBox.warning(self, "提示", "急停锁定未解除，请先点击“复位报警”解锁")
            return
        if self.vision_pallet_running:
            self.append_log("[提示] 视觉码垛已在运行中")
            return

        # 检查视觉标定是否完成
        vision_idx = int(self.vision_index_combo.currentText().split()[-1])
        if not self.calibration_status.get(vision_idx, False):
            self.append_log(f"[错误] 视觉工艺 {vision_idx} 尚未标定，请先完成标定")
            QMessageBox.warning(self, "提示", f"视觉工艺 {vision_idx} 未标定，请先在“视觉标定”标签中完成标定。")
            return

        # 检查机器人状态
        handle = self.robot_worker.handle
        try:
            with X5_LOCK:
                state = x5.get_system_state(handle)
                alarm = state.alarm
                enable = state.enable
                mode = state.mode
            if alarm:
                self.append_log("[错误] 机器人有报警")
                QMessageBox.warning(self, "提示", "机器人有报警，请先复位报警")
                return
            if not enable:
                self.append_log("[提示] 上使能中...")
                with X5_LOCK:
                    x5.enable_servo(handle, True)
                time.sleep(0.5)
                with X5_LOCK:
                    state = x5.get_system_state(handle)
                    enable = state.enable
                if not enable:
                    self.append_log("[错误] 上使能失败")
                    QMessageBox.warning(self, "提示", "上使能失败")
                    return
            if mode not in (2, 100):
                self.append_log("[提示] 切换到自动命令模式...")
                with X5_LOCK:
                    x5.set_system_mode(handle, 100)
                time.sleep(0.5)
                with X5_LOCK:
                    state = x5.get_system_state(handle)
                if state.mode != 100:
                    self.append_log("[错误] 切换模式失败")
                    QMessageBox.warning(self, "提示", "切换模式失败")
                    return
        except Exception as e:
            self.append_log(f"[错误] 状态检查异常: {e}")
            QMessageBox.warning(self, "提示", f"状态检查失败: {e}")
            return

        # 一次性捕获线程内需要用到的参数（避免在子线程中直接读取 Qt 控件）
        pallet_points = list(self.pallet_points)
        total_steps = len(pallet_points)
        photo_point = self.photo_point
        pallet_uf = self.pallet_uf
        pallet_tf = self.pallet_tf
        pallet_cfg = self.pallet_cfg
        move_type = self.move_type_combo.currentText()
        try:
            approach_offset = float(self.approach_offset_edit.text())
            depart_offset = float(self.depart_offset_edit.text())
        except ValueError:
            approach_offset = 0.0
            depart_offset = 0.0
        use_manual_c = self.use_manual_c_check.isChecked()
        try:
            manual_c = float(self.grab_c_edit.text())
        except ValueError:
            manual_c = 0.0

        # 重置进度
        self.current_step = 0
        self.vision_pallet_stop = False
        self.vision_pallet_running = True
        self.vision_pallet_btn.setEnabled(False)
        self.stop_vision_pallet_btn.setEnabled(True)
        self.append_log("[视觉码垛] 开始执行...")

        import threading
        def vision_pallet_thread():
            try:
                while self.vision_pallet_running and not self.vision_pallet_stop:
                    if self.robot_worker.handle is None:
                        self.append_log("[视觉码垛] 机器人连接已断开，停止")
                        break
                    if self.current_step >= total_steps:
                        self.append_log("[视觉码垛] 所有码垛点已用完，结束")
                        break

                    # 1. 移动到拍照点并等待真正到位（保证拍照位置准确）
                    self.append_log("[视觉码垛] 移动到拍照点")
                    self.robot_worker.movj_to_pose(photo_point.pose, photo_point.uf,
                                                   photo_point.tf, photo_point.cfg)
                    ok, reason = self.robot_worker.wait_idle(timeout=20.0, target=photo_point.pose,
                                                             stop_check=lambda: self.vision_pallet_stop)
                    if not ok:
                        self.append_log(f"[视觉码垛] 未到达拍照点: {reason}")
                        break
                    if self.vision_pallet_stop or not self.vision_pallet_running:
                        break
                    time.sleep(0.2)  # 留出相机/视觉稳定时间

                    # 2. 触发视觉拍照，获取所有工件点（同步调用）
                    self.append_log("[视觉码垛] 触发视觉拍照...")
                    received_points = self.vm_thread.trigger_and_get_sync(self.vm_thread.timeout + 1.0)
                    if not received_points:
                        self.append_log("[视觉码垛] 拍照超时或数据为空，等待后重试...")
                        time.sleep(0.5)
                        continue

                    self.append_log(f"[视觉码垛] 检测到 {len(received_points)} 个工件")

                    # 计算剩余码垛点数
                    remaining = total_steps - self.current_step
                    if len(received_points) > remaining:
                        self.append_log(f"[视觉码垛] 工件数({len(received_points)})多于剩余码垛点({remaining})，将只处理前{remaining}个")
                        received_points = received_points[:remaining]

                    # 3. 逐个处理每个工件
                    for idx, pixel in enumerate(received_points):
                        if self.vision_pallet_stop or not self.vision_pallet_running:
                            break
                        if self.current_step >= total_steps:
                            break

                        self.append_log(f"[视觉码垛] 处理工件 {idx+1}/{len(received_points)}")

                        # 视觉转换（动态转换）
                        trig_pose = photo_point.pose
                        trig_point = x5.Point(pose=trig_pose, uf=photo_point.uf,
                                              tf=photo_point.tf, cfg=photo_point.cfg)
                        pixel_pose = x5.Pose(pixel['x'], pixel['y'], 0, 0, 0, pixel['c'], 0, 0, 0)
                        try:
                            with X5_LOCK:
                                result_pose = x5.vision_dynamic_cnvrt(self.robot_worker.handle, vision_idx,
                                                                      pixel_pose, trig_point)
                        except Exception as e:
                            self.append_log(f"[视觉码垛] 视觉转换失败: {e}")
                            continue
                        # 是否用手动 C 覆盖视觉转换出的 C（避免抓取时 J4 一直转）
                        grab_c = manual_c if use_manual_c else result_pose.c
                        self.append_log(f"[视觉码垛] 转换后 X={result_pose.x:.2f} Y={result_pose.y:.2f} "
                                        f"C(视觉)={result_pose.c:.2f} C(使用)={grab_c:.2f}")

                        # 移动到抓取点并等待到位
                        grab_pose = x5.Pose(result_pose.x, result_pose.y, photo_point.pose.z,
                                            photo_point.pose.a, photo_point.pose.b,
                                            grab_c, 0, 0, 0)
                        self.append_log("[视觉码垛] 移动到抓取点")
                        self.robot_worker.movl_to_pose(grab_pose, photo_point.uf,
                                                       photo_point.tf, photo_point.cfg)
                        ok, reason = self.robot_worker.wait_idle(timeout=25.0, target=grab_pose,
                                                                 stop_check=lambda: self.vision_pallet_stop)
                        if not ok:
                            self.append_log(f"[视觉码垛] 未到达抓取点: {reason}")
                            self.vision_pallet_stop = True
                            break
                        if self.vision_pallet_stop or not self.vision_pallet_running:
                            break

                        # 移动到当前码垛点（带趋近/离开）
                        target_pose = pallet_points[self.current_step]
                        approach_z = target_pose.z + approach_offset
                        approach_pose = x5.Pose(target_pose.x, target_pose.y, approach_z,
                                                target_pose.a, target_pose.b, target_pose.c, 0, 0, 0)
                        depart_z = target_pose.z + depart_offset
                        depart_pose = x5.Pose(target_pose.x, target_pose.y, depart_z,
                                              target_pose.a, target_pose.b, target_pose.c, 0, 0, 0)

                        if move_type == "MOVJ":
                            self.robot_worker.movj_to_pose(approach_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)
                            self.robot_worker.movj_to_pose(target_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)
                            self.robot_worker.movj_to_pose(depart_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)
                        else:
                            self.robot_worker.movl_to_pose(approach_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)
                            self.robot_worker.movl_to_pose(target_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)
                            self.robot_worker.movl_to_pose(depart_pose, pallet_uf,
                                                           pallet_tf, pallet_cfg)

                        self.current_step += 1
                        QMetaObject.invokeMethod(self, "update_progress", Qt.QueuedConnection,
                                                 Q_ARG(int, self.current_step),
                                                 Q_ARG(int, total_steps),
                                                 Q_ARG(float, target_pose.x),
                                                 Q_ARG(float, target_pose.y),
                                                 Q_ARG(float, target_pose.z))

                    if self.current_step >= total_steps:
                        self.append_log("[视觉码垛] 所有码垛点已完成")
                        break

                    self.append_log("[视觉码垛] 准备下一次拍照...")
                    time.sleep(0.3)

            except Exception as e:
                import traceback
                error_msg = traceback.format_exc()
                self.append_log(f"[视觉码垛] 发生严重异常: {error_msg}")
            finally:
                self.vision_pallet_running = False
                QMetaObject.invokeMethod(self, "on_vision_pallet_finished", Qt.QueuedConnection)

        threading.Thread(target=vision_pallet_thread, daemon=True).start()

    @pyqtSlot()
    def on_vision_pallet_finished(self):
        self.vision_pallet_btn.setEnabled(True)
        self.stop_vision_pallet_btn.setEnabled(False)
        if not self.vision_pallet_stop:
            self.append_log("[视觉码垛] 已完成所有码垛点")
        else:
            self.append_log("[视觉码垛] 已停止")

    def on_stop_vision_pallet(self):
        self.vision_pallet_stop = True
        self.vision_pallet_running = False
        self.stop_vision_pallet_btn.setEnabled(False)
        self.vision_pallet_btn.setEnabled(True)
        self.robot_worker.abort_immediately(lock_motion=False)
        self.append_log("[视觉码垛] 用户请求停止，机器人已停止")

    # ===== 日志 =====
    def clear_log(self):
        self.log_text.clear()

    def closeEvent(self, event):
        self.state_timer.stop()
        self.vm_thread.stop()
        self.vm_thread.quit()
        self.vm_thread.wait()
        self.robot_worker.stop()
        self.robot_worker.quit()
        self.robot_worker.wait()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())