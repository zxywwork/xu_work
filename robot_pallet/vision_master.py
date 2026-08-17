# -*- coding: utf-8 -*-
"""
Vision Master 通信线程。

职责：
  - 通过 TCP socket 连接 Vision Master；
  - 支持异步触发（_do_trigger，用于界面手动拍照）与同步触发
    （trigger_and_get_sync，用于视觉码垛子线程）；
  - 解析 "x,y,c,attr,id;..." 格式的视觉数据为点列表。

线程安全：
  - sock_lock 保护 socket 收发，避免界面触发与码垛线程同步触发并发读写。
"""
import socket
import time
from threading import Lock

from PyQt5.QtCore import QThread, pyqtSignal


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

    @staticmethod
    def _parse_data(raw):
        """解析 "x,y,c[,attr[,id]]; x,y,c,..." 字符串为点字典列表"""
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
