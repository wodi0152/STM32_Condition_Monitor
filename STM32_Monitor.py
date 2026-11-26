#!/usr/bin/env python3
"""STM32 Condition Monitor: live telemetry from STM32 via USB-RS485.

Protocol (115200 8N1; one ASCII line every 500 ms):
MON,uptime_ms,pwm_percent,bus_mV,current_mA,temp_x10,rms_mg,fifo_overflows

A missing sensor value may be sent as NA.
Install: python -m pip install pyserial
Run:     python STM32_Condition_Monitor_PC.py
"""

import csv
import queue
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("Missing pyserial. Install it with: py -m pip install pyserial") from exc


BG = "#10151f"
PANEL = "#1b2431"
CARD = "#202c3a"
EDGE = "#354355"
FG = "#ebf2fc"
MUTED = "#9bacc0"
ACCENT = "#6cd9c4"
YELLOW = "#f5cf77"
RED = "#ef8c8c"


def parse_packet(line):
    """Return a validated measurement dictionary or None.

    MON,12000,50,6000,100,224,56,0
    """
    fields = line.strip().split(",")
    if len(fields) != 8 or fields[0] != "MON":
        return None

    def number(x):
        x = x.strip()
        if x in ("NA", "-", ""):
            return None
        return int(x)

    try:
        elapsed_ms = int(fields[1])
        pwm = int(fields[2])
        bus = number(fields[3])
        current = number(fields[4])
        temp10 = number(fields[5])
        rms = number(fields[6])
        overflows = number(fields[7])
        if elapsed_ms < 0 or not 0 <= pwm <= 100:
            return None
        return {
            "uptime_ms": elapsed_ms,
            "pwm": pwm,
            "bus_mv": bus,
            "current_ma": current,
            "temp_c": temp10 / 10 if temp10 is not None else None,
            "rms_mg": rms,
            "overflows": overflows,
        }
    except (ValueError, OverflowError):
        return None


def text_value(value, unit="", decimals=0):
    if value is None:
        return "—"
    if decimals:
        return f"{value:.{decimals}f}{unit}"
    return f"{value}{unit}"


class MonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Condition Monitor | RS485 (English)")
        self.root.geometry("1100x725")
        self.root.minsize(820, 620)
        self.root.configure(bg=BG)

        self.ser = None
        self.reader_thread = None
        self.reader_stop = threading.Event()
        self.messages = queue.Queue()
        self.last_rx = 0.0
        self.connected_at = 0.0
        self.packet_count = 0
        self.csv_handle = None
        self.csv_writer = None

        self.history_current = deque(maxlen=120)
        self.history_rms = deque(maxlen=120)

        self.fields = {
            "pwm": tk.StringVar(value="—"),
            "bus": tk.StringVar(value="—"),
            "current": tk.StringVar(value="—"),
            "temp": tk.StringVar(value="—"),
            "rms": tk.StringVar(value="—"),
            "overflows": tk.StringVar(value="—"),
        }
        self.status = tk.StringVar(value="Disconnected · select the USB–RS485 COM port")
        self.count_str = tk.StringVar(value="Packets: 0")
        self.port_var = tk.StringVar()
        self.build_ui()
        self.refresh_ports()
        self.root.after(100, self.process_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def build_ui(self):
        container = tk.Frame(self.root, bg=BG, padx=23, pady=20)
        container.pack(fill="both", expand=True)
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(3, weight=1)

        heading = tk.Frame(container, bg=BG)
        heading.grid(row=0, column=0, sticky="ew", pady=(0, 18))
        tk.Label(heading, text="STM32  /  CONDITION MONITOR", bg=BG, fg=FG,
                 font=("Segoe UI", 21, "bold")).pack(anchor="w")
        tk.Label(heading, text="N20 Motor Monitoring · INA219 · DS18B20 · MPU6050 · RS485",
                 bg=BG, fg=MUTED, font=("Segoe UI", 10)).pack(anchor="w", pady=(4, 0))

        controls = tk.Frame(container, bg=PANEL, padx=14, pady=12,
                            highlightbackground=EDGE, highlightthickness=1)
        controls.grid(row=1, column=0, sticky="ew", pady=(0, 14))

        tk.Label(controls, text="COM Port", bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 10)).pack(side="left", padx=(0, 8))
        self.ports = ttk.Combobox(controls, textvariable=self.port_var,
                                  width=27, state="readonly")
        self.ports.pack(side="left", padx=(0, 8))
        self.make_button(controls, "Refresh", self.refresh_ports, PANEL, FG).pack(
            side="left", padx=(0, 8))
        self.connect_btn = self.make_button(controls, "Connect", self.toggle_connection,
                                            ACCENT, "#12232a")
        self.connect_btn.pack(side="left", padx=(0, 9))
        self.record_btn = self.make_button(controls, "Record CSV", self.toggle_record,
                                           YELLOW, "#1b2431")
        self.record_btn.pack(side="left")

        cards = tk.Frame(container, bg=BG)
        cards.grid(row=2, column=0, sticky="ew", pady=(0, 14))
        for i in range(3):
            cards.grid_columnconfigure(i, weight=1, uniform="cards")
        labels = [
            ("PWM", "pwm", "PWM duty cycle"),
            ("CURRENT", "current", "INA219 · motor driver supply"),
            ("VOLTAGE", "bus", "INA219 · BUS"),
            ("TEMPERATURE", "temp", "DS18B20"),
            ("VIBRATION RMS", "rms", "MPU6050 · mg"),
            ("FIFO OVERFLOWS", "overflows", "FIFO buffer overflows"),
        ]
        for n, (title, field, description) in enumerate(labels):
            card = tk.Frame(cards, bg=CARD, padx=18, pady=13,
                            highlightbackground=EDGE, highlightthickness=1)
            card.grid(row=n // 3, column=n % 3, sticky="nsew",
                      padx=(0 if n % 3 == 0 else 6, 0), pady=(0, 8))
            tk.Label(card, text=title, bg=CARD, fg=MUTED,
                     font=("Segoe UI", 9, "bold")).pack(anchor="w")
            tk.Label(card, textvariable=self.fields[field], bg=CARD, fg=ACCENT,
                     font=("Segoe UI", 24, "bold")).pack(anchor="w", pady=(2, 2))
            tk.Label(card, text=description, bg=CARD, fg=MUTED,
                     font=("Segoe UI", 9)).pack(anchor="w")

        graphs = tk.Frame(container, bg=BG)
        graphs.grid(row=3, column=0, sticky="nsew")
        graphs.grid_columnconfigure(0, weight=1)
        graphs.grid_columnconfigure(1, weight=1)
        graphs.grid_rowconfigure(0, weight=1)
        self.current_canvas = self.create_chart(graphs, 0, "CURRENT (mA)", YELLOW)
        self.rms_canvas = self.create_chart(graphs, 1, "VIBRATION RMS (mg)", ACCENT)
        for canvas in (self.current_canvas, self.rms_canvas):
            canvas.bind("<Configure>", lambda event: self.redraw_graphs())

        footer = tk.Frame(container, bg=BG)
        footer.grid(row=4, column=0, sticky="ew", pady=(14, 0))
        tk.Label(footer, textvariable=self.status, bg=BG, fg=FG,
                 font=("Segoe UI", 10)).pack(side="left")
        tk.Label(footer, textvariable=self.count_str, bg=BG, fg=MUTED,
                 font=("Segoe UI", 9)).pack(side="right")
        self.redraw_graphs()

    def make_button(self, parent, title, callback, bg, fg):
        return tk.Button(parent, text=title, command=callback, bg=bg, fg=fg,
                         activebackground=ACCENT, activeforeground="#14232d",
                         relief="flat", bd=0, padx=12, pady=8,
                         font=("Segoe UI", 9, "bold"), cursor="hand2")

    def create_chart(self, parent, col, heading, color):
        panel = tk.Frame(parent, bg=PANEL, highlightbackground=EDGE,
                         highlightthickness=1, padx=12, pady=10)
        panel.grid(row=0, column=col, sticky="nsew", padx=(0, 6) if col == 0 else (6, 0))
        tk.Label(panel, text=heading, bg=PANEL, fg=FG,
                 font=("Segoe UI", 11, "bold")).pack(anchor="w")
        tk.Label(panel, text="Last ~60 seconds · auto scale",
                 bg=PANEL, fg=MUTED, font=("Segoe UI", 9)).pack(anchor="w", pady=(2, 6))
        canvas = tk.Canvas(panel, bg=PANEL, highlightthickness=0, height=205)
        canvas.pack(fill="both", expand=True)
        canvas.line_color = color
        return canvas

    def refresh_ports(self):
        try:
            ports = list(list_ports.comports())
            names = [f"{port.device}  —  {port.description}" for port in ports]
            self.ports["values"] = names
            if names and not self.port_var.get() in names:
                self.port_var.set(names[0])
            if not names and self.ser is None:
                self.port_var.set("")
                self.status.set("No COM ports found · connect the USB–RS485 adapter")
        except Exception as error:
            self.status.set(f"Port scan error: {error}")

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
            return
        selected = self.port_var.get()
        if not selected:
            messagebox.showwarning("No COM port selected", "Connect your USB–RS485 adapter and click Refresh.")
            return
        port = selected.split("  —  ", 1)[0]
        try:
            self.ser = serial.Serial(port=port, baudrate=115200,
                                     bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE, timeout=0.3)
            self.reader_stop.clear()
            self.reader_thread = threading.Thread(target=self.serial_reader,
                                                  args=(self.ser,), daemon=True)
            self.reader_thread.start()
            self.connect_btn.configure(text="Disconnect", bg=RED)
            self.status.set(f"{port}: port open · waiting for STM32…")
            self.last_rx = 0
            self.connected_at = time.monotonic()
        except (serial.SerialException, OSError) as error:
            self.ser = None
            messagebox.showerror("Could not open COM port", str(error))

    def serial_reader(self, conn):
        while not self.reader_stop.is_set():
            try:
                raw = conn.readline()
                if raw:
                    message = raw.decode("ascii", errors="replace").strip()
                    self.messages.put(("line", message))
            except (serial.SerialException, OSError) as error:
                if not self.reader_stop.is_set():
                    self.messages.put(("error", str(error)))
                break

    def disconnect(self):
        self.reader_stop.set()
        old, self.ser = self.ser, None
        if old is not None:
            try:
                old.close()
            except (serial.SerialException, OSError):
                pass
        self.connect_btn.configure(text="Connect", bg=ACCENT)
        self.status.set("Disconnected")

    def toggle_record(self):
        if self.csv_handle:
            self.stop_record()
            return
        default = f"stm32_monitor_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(title="Save Measurements",
                                            defaultextension=".csv", initialfile=default,
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            self.csv_handle = open(path, "w", newline="", encoding="utf-8-sig")
            self.csv_writer = csv.writer(self.csv_handle)
            self.csv_writer.writerow(["received_at", "uptime_ms", "pwm_percent",
                                      "bus_mv", "current_ma", "temperature_c",
                                      "rms_mg", "fifo_overflows"])
            self.record_btn.configure(text="Stop Recording", bg=RED)
            self.status.set(f"Recording CSV: {Path(path).name}")
        except OSError as error:
            messagebox.showerror("CSV Error", str(error))
            self.csv_handle = None
            self.csv_writer = None

    def stop_record(self):
        if self.csv_handle is not None:
            try:
                self.csv_handle.close()
            except OSError:
                pass
        self.csv_handle = None
        self.csv_writer = None
        self.record_btn.configure(text="Record CSV", bg=YELLOW)
        self.status.set("CSV file saved")

    def process_queue(self):
        processed = 0
        while processed < 200:
            try:
                kind, payload = self.messages.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if kind == "error":
                self.disconnect()
                self.status.set(f"Connection lost: {payload}")
                continue
            if self.ser is None:
                continue
            packet = parse_packet(payload)
            if packet is not None:
                self.on_packet(packet)

        if self.ser is not None and time.monotonic() - (self.last_rx or self.connected_at) > 3:
            self.status.set("No data for over 3 s · check RS485 A/B and EN")
        self.root.after(100, self.process_queue)

    def on_packet(self, data):
        self.last_rx = time.monotonic()
        self.packet_count += 1
        self.count_str.set(f"Packets: {self.packet_count}")
        self.fields["pwm"].set(text_value(data["pwm"], "%"))
        self.fields["bus"].set(text_value(data["bus_mv"] / 1000 if data["bus_mv"] is not None else None,
                                          " V", decimals=3))
        self.fields["current"].set(text_value(data["current_ma"], " mA"))
        self.fields["temp"].set(text_value(data["temp_c"], " °C", decimals=1))
        self.fields["rms"].set(text_value(data["rms_mg"], " mg"))
        self.fields["overflows"].set(text_value(data["overflows"]))
        self.history_current.append(data["current_ma"])
        self.history_rms.append(data["rms_mg"])
        self.redraw_graphs()
        port = self.ser.port if self.ser is not None else ""
        self.status.set(f"{port} · 115200 8N1 · receiving data every ~500 ms")
        if self.csv_writer is not None:
            try:
                self.csv_writer.writerow([
                    datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    data["uptime_ms"], data["pwm"], data["bus_mv"],
                    data["current_ma"], data["temp_c"], data["rms_mg"],
                    data["overflows"],
                ])
                self.csv_handle.flush()
            except OSError as error:
                self.stop_record()
                self.status.set(f"CSV write error: {error}")

    def redraw_graphs(self):
        self.draw_one_graph(self.current_canvas, self.history_current)
        self.draw_one_graph(self.rms_canvas, self.history_rms)

    def draw_one_graph(self, canvas, values):
        canvas.delete("all")
        width = max(canvas.winfo_width(), 350)
        height = max(canvas.winfo_height(), 175)
        left, top, right, bottom = 42, 15, width - 15, height - 28
        actual = [x for x in values if x is not None]
        if actual:
            low = min(0, min(actual))
            high = max(10, max(actual))
            if high == low:
                high += 1
            padding = max(1.0, (high - low) * 0.08)
            lo, hi = low - padding, high + padding
        else:
            lo, hi = 0, 100

        for line in range(5):
            fraction = line / 4
            y = top + fraction * (bottom - top)
            canvas.create_line(left, y, right, y, fill=EDGE, dash=(2, 4))
            canvas.create_text(left - 7, y, text=f"{hi - fraction * (hi - lo):.0f}",
                               fill=MUTED, anchor="e", font=("Segoe UI", 8))
        canvas.create_text(right, bottom + 16, text="now", fill=MUTED,
                           anchor="e", font=("Segoe UI", 8))
        if len(values) < 2:
            canvas.create_text((left + right) / 2, (top + bottom) / 2,
                               text="Waiting for telemetry…", fill=MUTED,
                               font=("Segoe UI", 10))
            return

        points = []
        items = list(values)
        for index, value in enumerate(items):
            if value is None:
                if len(points) >= 4:
                    canvas.create_line(*points, fill=canvas.line_color, width=2.5, smooth=False)
                points = []
                continue
            x = left + index * (right - left) / max(1, len(items) - 1)
            y = bottom - (value - lo) * (bottom - top) / (hi - lo)
            points.extend([x, y])
        if len(points) >= 4:
            canvas.create_line(*points, fill=canvas.line_color, width=2.5)

    def close(self):
        self.stop_record() if self.csv_handle else None
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = MonitorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
