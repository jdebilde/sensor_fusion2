"""
sudo apt-get install python3-tk
sudo apt-get install python3-pil python3-pil.imagetk
"""
import struct
import queue
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from CsvReplaySource import CsvReplaySource
from LiveUdpSource import LiveUdpSource
from config import MCAST_GRP, MCAST_PORT
from HeraSample import FORMATS
from views.Nav2 import Nav2
from views.Nav1_acc import Nav1_acc
from views.Nav1_gyro import Nav1_gyro
from views.Nav1_mag import Nav1_mag
from views.Nav2_ekf import Nav2_ekf
from views.Heading_gyro import Heading_gyro
from views.Ekf_v1 import Ekf_v1
from views.Position_stevin import Position_stevin
from views.Position_stevin2 import Position_stevin2

DEFAULT_PC_IP = "172.20.10.3"
DEFAULT_MEASURE = "nav2"
DEFAULT_NODE = "sensor_fusion@nav_3"

VIEWS = {
    Nav1_acc.name: Nav1_acc,
    Nav1_gyro.name: Nav1_gyro,
    Nav1_mag.name: Nav1_mag,
    Nav2.name: Nav2,
    Nav2_ekf.name: Nav2_ekf,
    Position_stevin.name: Position_stevin,
    Position_stevin2.name: Position_stevin2,
    Heading_gyro.name: Heading_gyro,
    Ekf_v1.name: Ekf_v1,
}

MEASUREMENTS = list(VIEWS.keys()) + ['ekf_v2'] + ['bilateration'] + ['ekf2_uwb'] + ['ekf4_nav2'] + ['ekf4_nav2_uwb'] + ['ekf6_nav2'] + ['ekf6_nav2_uwb'] + ['nav2_acc_gyro']

class HeraViewerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Hera Live Viewer")

        self.queue = queue.Queue()
        self.source = None
        self.view = None

        self.last_redraw = 0

        self.build_ui()
        self.create_view()
        self.root.after(50, self.process_queue)

    def build_ui(self):
        controls = ttk.Frame(self.root, padding=8)
        controls.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(controls, text="Source").grid(row=0, column=0, sticky="w")
        self.source_var = tk.StringVar(value="Live UDP")
        self.source_combo = ttk.Combobox(
            controls,
            textvariable=self.source_var,
            values=["Live UDP", "CSV"],
            width=12,
            state="readonly"
        )
        self.source_combo.grid(row=0, column=1, padx=4)

        ttk.Label(controls, text="PC IP").grid(row=0, column=2, sticky="w")
        self.pc_ip_var = tk.StringVar(value=DEFAULT_PC_IP)
        ttk.Entry(controls, textvariable=self.pc_ip_var, width=15).grid(row=0, column=3, padx=4)

        ttk.Label(controls, text="Measure").grid(row=0, column=4, sticky="w")
        self.measure_var = tk.StringVar(value=DEFAULT_MEASURE)
        self.view_combo = ttk.Combobox(
            controls,
            textvariable=self.measure_var,
            values=MEASUREMENTS,
            width=25,
            state="readonly"
        )
        self.view_combo.grid(row=0, column=5, columnspan=1, padx=4, sticky="w")
        self.view_combo.bind("<<ComboboxSelected>>", lambda event: self.create_view())

        ttk.Label(controls, text="Node").grid(row=0, column=6, sticky="w")
        self.node_var = tk.StringVar(value=DEFAULT_NODE)
        ttk.Entry(controls, textvariable=self.node_var, width=25).grid(row=0, column=7, padx=4)

        ttk.Label(controls, text="Vue").grid(row=1, column=0, sticky="w", pady=6)
        self.view_var = tk.StringVar(value=Nav2.name)
        self.view_combo = ttk.Combobox(
            controls,
            textvariable=self.view_var,
            values=list(VIEWS.keys()),
            width=25,
            state="readonly"
        )
        self.view_combo.grid(row=1, column=1, columnspan=2, padx=4, sticky="w")
        self.view_combo.bind("<<ComboboxSelected>>", lambda event: self.create_view())

        self.csv_file_var = tk.StringVar(value="")
        ttk.Entry(controls, textvariable=self.csv_file_var, width=45).grid(
            row=1, column=3, columnspan=4, padx=4, sticky="we"
        )
        ttk.Button(controls, text="Select CSV", command=self.choose_csv).grid(row=1, column=7, padx=4)

        self.start_button = ttk.Button(controls, text="Start", command=self.start)
        self.start_button.grid(row=0, column=8, padx=4)

        self.stop_button = ttk.Button(controls, text="Stop", command=self.stop)
        self.stop_button.grid(row=1, column=8, padx=4)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var, padding=6).pack(side=tk.BOTTOM, fill=tk.X)

        self.view_frame = ttk.Frame(self.root)
        self.view_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # ttk.Label(controls, text="Format").grid(row=2, column=0, sticky="w")
        self.format_var = tk.StringVar(value="nav2")
        # self.format_combo = ttk.Combobox(
        #     controls,
        #     textvariable=self.format_var,
        #     values=list(FORMATS.keys()),
        #     width=15,
        #     state="readonly"
        # )
        # self.format_combo.grid(row=2, column=1, padx=4)

        ttk.Label(controls, text="CSV mode").grid(row=2, column=0, sticky="w")

        # read csv in replay or full mode
        self.csv_mode_var = tk.StringVar(value="replay")
        self.csv_mode_combo = ttk.Combobox(
            controls,
            textvariable=self.csv_mode_var,
            values=["replay", "full"],
            width=10,
            state="readonly"
        )
        self.csv_mode_combo.grid(row=2, column=1, padx=4, sticky="w")

        ttk.Button(
            controls,
            text="Export PNG",
            command=self.export_current_view
        ).grid(row=2, column=8, padx=4)

    def choose_csv(self):
        filename = filedialog.askopenfilename(
            title="Select a Hera CSV file",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if filename:
            self.csv_file_var.set(filename)

    def create_view(self):
        for child in self.view_frame.winfo_children():
            child.destroy()

        view_name = self.view_var.get()
        view_cls = VIEWS[view_name]

        if hasattr(view_cls, "default_format"):
            self.format_var.set(view_cls.default_format)

        self.view = view_cls(self.view_frame)

    def start(self):
        self.stop()

        measure = self.measure_var.get().strip()
        node = self.node_var.get().strip()
        format_name = self.format_var.get()

        if not measure:
            messagebox.showerror("Error", "Empty Measure")
            return

        if not node:
            messagebox.showerror("Error", "Empty node")
            return

        source_type = self.source_var.get()

        if source_type == "Live UDP":
            pc_ip = self.pc_ip_var.get().strip()
            self.source = LiveUdpSource(
                self.queue,
                pc_ip=pc_ip,
                measure_filter=measure,
                node_filter=node,
                format_name=format_name
            )
            self.status_var.set(f"Live UDP: {MCAST_GRP}:{MCAST_PORT}, measure={measure}, node={node}")

        elif source_type == "CSV":
            filename = self.csv_file_var.get().strip()
            if not filename:
                messagebox.showerror("Error", "Select a CSV file")
                return

            self.source = CsvReplaySource(
                self.queue,
                filename=filename,
                measure_filter=measure,
                node_filter=node,
                format_name=format_name,
                replay_speed=2.0,
                mode=self.csv_mode_var.get()
            )
            self.status_var.set(f"Reading CSV: {filename}")

        else:
            messagebox.showerror("Error", f"Unknown source: {source_type}")
            return

        self.create_view()
        self.source.start()

    def stop(self):
        if self.source:
            self.source.stop()
            self.source = None

        self.status_var.set("Stopped")

    def process_queue(self):
        updated = False

        while True:
            try:
                kind, payload = self.queue.get_nowait()
            except queue.Empty:
                break

            if kind == "sample":
                self.view.add_sample(payload)
                updated = True

            elif kind == "samples":
                self.view.add_samples(payload)
                updated = True

            elif kind == "error":
                self.status_var.set(payload)

            elif kind == "warning":
                print(payload)

        now = time.time()

        # avoid redrawing too often
        if updated and now - self.last_redraw > 0.1:
            self.view.redraw()
            self.last_redraw = now

        self.root.after(50, self.process_queue)
    
    def export_current_view(self):
        if self.view is None:
            return

        self.view.export_subplots(
            output_dir="exports",
            prefix=self.view.name
        )


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1200x800")
    app = HeraViewerApp(root)
    root.mainloop()