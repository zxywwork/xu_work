# -*- coding: utf-8 -*-
"""
机器人 API 调用线程（RobotWorker）。

职责：
  - 通过命令队列 + cmd_lock 串行调用 xapi（x5 / x5v）的机器人 API，
    避免主线程 / 码垛线程 / 状态刷新并发调用 SDK；
  - 支持连接、状态查询、模式/使能/报警复位、速度、MOVJ/MOVL、
    视觉配置写入、自动标定、动态转换验证、UF/TF 切换、PR 写入等；
  - wait_idle：等待命令队列清空且机器人真正到位，供视觉码垛等流程调用。

线程安全 / 约定：
  - run() 的每个命令分发包在 `with X5_LOCK:` 内调用原生 SDK；
  - 急停：abort_immediately(lock_motion=True) 清空队列 + x5.abort + 置
    _stop_flag；_stop_flag 为 True 时丢弃运动命令，复位报警或重连后解锁。
"""
import math
import time
import traceback
from threading import Lock

from PyQt5.QtCore import QThread, pyqtSignal

import xapi.api as x5
import xapi.api.vision as x5v

from .common import X5_LOCK


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
            from .common import MODE_NAMES
            self.log_signal.emit(f"[模式] -> {MODE_NAMES.get(mode, str(mode))}")
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
