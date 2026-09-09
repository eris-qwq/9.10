#!/usr/bin/env python3
"""STM32 USB CDC 航向角漂移监视器，仅使用 Python 标准库。"""

import glob
import os
import select
import struct
import termios
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def serial_ports():
    paths = glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
    result = []
    for path in paths:
        real = os.path.realpath(path)
        if real not in [os.path.realpath(p) for p in result]:
            result.append(path)
    return result


class Monitor:
    def __init__(self, root):
        self.root = root
        root.title("航向角漂移监视器")
        root.geometry("900x680")
        root.minsize(720, 520)
        self.fd = None
        self.buffer = bytearray()
        self.samples = deque(maxlen=3000)
        self.zero = None
        self.last_rx = 0.0

        bar = ttk.Frame(root, padding=(10, 10, 10, 4))
        bar.pack(fill="x")
        ttk.Label(bar, text="STM32 数据口：").pack(side="left")
        self.port = ttk.Combobox(bar, state="readonly")
        self.port.pack(side="left", fill="x", expand=True, padx=(0, 8))

        buttons = ttk.Frame(root, padding=(10, 4, 10, 6))
        buttons.pack(fill="x")
        ttk.Button(buttons, text="刷新串口", command=self.refresh).pack(side="left")
        self.connect_btn = ttk.Button(buttons, text="连接", command=self.toggle)
        self.connect_btn.pack(side="left", padx=8)
        ttk.Button(buttons, text="当前角度设为零点", command=self.set_zero).pack(side="left")
        ttk.Button(buttons, text="清空重新测量", command=self.clear).pack(side="left", padx=8)

        self.verdict = tk.StringVar(value="正在自动连接…")
        self.verdict_label = tk.Label(root, textvariable=self.verdict, font=("Sans", 20, "bold"),
                                      bg="#666666", fg="white", pady=8)
        self.verdict_label.pack(fill="x", padx=10, pady=(0, 4))

        values = ttk.Frame(root, padding=(12, 4))
        values.pack(fill="x")
        self.angle_text = tk.StringVar(value="当前航向角\n--.--°")
        self.drift_text = tk.StringVar(value="从零点漂了\n--.--°")
        self.rate_text = tk.StringVar(value="当前角速度\n--.--°/s")
        self.range_text = tk.StringVar(value="60秒内摆动范围\n--.--°")
        for column, var in enumerate((self.angle_text, self.drift_text, self.rate_text, self.range_text)):
            ttk.Label(values, textvariable=var, font=("Sans", 14), anchor="center",
                      justify="center").grid(row=0, column=column, sticky="ew", padx=5)
            values.columnconfigure(column, weight=1)

        telemetry = ttk.Frame(root, padding=(12, 2))
        telemetry.pack(fill="x")
        self.temperature_text = tk.StringVar(value="IMU温度：--.-- ℃（目标40.00 ℃）")
        self.heater_text = tk.StringVar(value="加热PWM：---- / 9999（--.-%）")
        ttk.Label(telemetry, textvariable=self.temperature_text, font=("Sans", 13)).pack(side="left", expand=True)
        ttk.Label(telemetry, textvariable=self.heater_text, font=("Sans", 13)).pack(side="left", expand=True)

        pid_panel = ttk.LabelFrame(root, text="板子当前实际运行的航向 PID", padding=(10, 5))
        pid_panel.pack(fill="x", padx=10, pady=4)
        self.position_pid_text = tk.StringVar(value="角度环：Kp=--  Ki=--  Kd=--")
        self.speed_pid_text = tk.StringVar(value="速度环：Kp=--  Ki=--  Kd=--")
        ttk.Label(pid_panel, textvariable=self.position_pid_text, font=("Monospace", 13)).pack(side="left", expand=True)
        ttk.Label(pid_panel, textvariable=self.speed_pid_text, font=("Monospace", 13)).pack(side="left", expand=True)

        self.canvas = tk.Canvas(root, bg="#10151c", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=8)
        self.status = tk.StringVar(value="请连接 STM32 Type-C 数据口")
        ttk.Label(root, textvariable=self.status, padding=8).pack(fill="x")
        self.refresh()
        if self.port.get():
            root.after(300, self.toggle)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(20, self.poll)
        root.after(100, self.draw)

    def refresh(self):
        ports = serial_ports()
        self.port["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def toggle(self):
        if self.fd is not None:
            self.disconnect()
            return
        path = self.port.get()
        if not path:
            messagebox.showwarning("未找到串口", "请插入 STM32 Type-C USB，然后点击‘刷新串口’。")
            return
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
            attr = termios.tcgetattr(self.fd)
            attr[0] = attr[1] = attr[3] = 0
            attr[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attr[4] = attr[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
            self.connect_btn.config(text="断开")
            self.status.set(f"已连接 {path}，等待功能码 0xA0 的航向帧…")
            self.verdict.set("已连接，正在等待航向数据…")
        except OSError as exc:
            self.fd = None
            messagebox.showerror("连接失败", f"{exc}\n\n请关闭占用该串口的原上位机。")

    def disconnect(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.connect_btn.config(text="连接")
        self.status.set("已断开")

    def close(self):
        self.disconnect()
        self.root.destroy()

    def clear(self):
        self.samples.clear()
        self.zero = None

    def set_zero(self):
        if self.samples:
            self.zero = self.samples[-1][1]

    def poll(self):
        if self.fd is not None:
            try:
                if select.select([self.fd], [], [], 0)[0]:
                    chunk = os.read(self.fd, 4096)
                    self.buffer.extend(chunk)
                    self.parse()
            except OSError as exc:
                self.disconnect()
                self.status.set(f"串口已断开：{exc}")
        self.root.after(20, self.poll)

    def parse(self):
        while True:
            try:
                head = self.buffer.index(0x5A)
            except ValueError:
                self.buffer.clear()
                return
            if head:
                del self.buffer[:head]
            if len(self.buffer) < 6:
                return
            length = self.buffer[4] | self.buffer[5] << 8
            if length > 64:
                del self.buffer[0]
                continue
            total = 8 + length
            if len(self.buffer) < total:
                return
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            payload = frame[6:6 + length]
            received = frame[-2] | frame[-1] << 8
            if crc16(payload) != received:
                continue
            if frame[1] == 0xA0 and length in (10, 16, 40):
                yaw_cd, omega_cd, uptime = struct.unpack_from("<ihI", payload)
                angle = yaw_cd / 100.0
                rate = omega_cd / 100.0
                if self.zero is None:
                    self.zero = angle
                self.samples.append((uptime / 1000.0, angle, rate))
                self.last_rx = time.monotonic()
                if length >= 16:
                    temperature_cd, heater_pwm, target_cd = struct.unpack_from("<hHH", payload, 10)
                    self.temperature_text.set(
                        f"IMU温度：{temperature_cd/100.0:.2f} ℃（目标{target_cd/100.0:.2f} ℃）")
                    self.heater_text.set(
                        f"加热PWM：{heater_pwm} / 9999（{heater_pwm/99.99:.1f}%）")
                if length >= 40:
                    pkp, pki, pkd, skp, ski, skd = struct.unpack_from("<6f", payload, 16)
                    self.position_pid_text.set(
                        f"角度环：Kp={pkp:g}  Ki={pki:g}  Kd={pkd:g}")
                    self.speed_pid_text.set(
                        f"速度环：Kp={skp:g}  Ki={ski:g}  Kd={skd:g}")

    def draw(self):
        c = self.canvas
        c.delete("all")
        width = max(c.winfo_width(), 2)
        height = max(c.winfo_height(), 2)
        c.create_text(12, 12, anchor="nw", fill="#8d9aaa", text="最近60秒：相对启动零点的航向角")
        if self.samples:
            now = self.samples[-1][0]
            visible = [s for s in self.samples if now - s[0] <= 60.0]
            drift_values = [s[1] - self.zero for s in visible]
            low, high = min(drift_values), max(drift_values)
            span = max(high - low, 0.2)
            margin = span * 0.15
            low -= margin
            high += margin
            for i in range(5):
                y = 40 + i * (height - 70) / 4
                value = high - i * (high - low) / 4
                c.create_line(55, y, width - 10, y, fill="#27313d")
                c.create_text(50, y, anchor="e", fill="#8d9aaa", text=f"{value:.2f}°")
            points = []
            start = max(now - 60.0, visible[0][0])
            time_span = max(now - start, 1.0)
            for t, angle, _ in visible:
                x = 55 + (t - start) / time_span * (width - 65)
                y = 40 + (high - (angle - self.zero)) / (high - low) * (height - 70)
                points.extend((x, y))
            if len(points) >= 4:
                c.create_line(*points, fill="#35c7ff", width=2)
            angle, rate = self.samples[-1][1], self.samples[-1][2]
            drift = angle - self.zero
            peak_to_peak = max(drift_values) - min(drift_values)
            self.angle_text.set(f"当前航向角\n{angle:.2f}°")
            self.drift_text.set(f"从零点漂了\n{drift:+.2f}°")
            self.rate_text.set(f"当前角速度\n{rate:+.2f}°/s")
            self.range_text.set(f"60秒内摆动范围\n{peak_to_peak:.2f}°")
            elapsed = visible[-1][0] - visible[0][0]
            if elapsed < 5.0:
                self.verdict.set(f"正在收集数据，再静置 {5.0-elapsed:.0f} 秒…")
                self.verdict_label.config(bg="#666666")
            elif abs(drift) <= 0.5 and peak_to_peak <= 1.0:
                self.verdict.set("结论：稳定，暂未发现明显漂移")
                self.verdict_label.config(bg="#198754")
            elif abs(drift) <= 2.0 and peak_to_peak <= 3.0:
                self.verdict.set("结论：有轻微漂移，建议继续静置观察")
                self.verdict_label.config(bg="#b07800")
            else:
                self.verdict.set("结论：漂移明显，需检查 IMU 校准和机械震动")
                self.verdict_label.config(bg="#b02a37")
            age = time.monotonic() - self.last_rx
            self.status.set("数据正常（50 Hz）" if age < 0.5 else f"已 {age:.1f} 秒未收到航向帧")
        self.root.after(100, self.draw)


if __name__ == "__main__":
    app = tk.Tk()
    Monitor(app)
    app.mainloop()
