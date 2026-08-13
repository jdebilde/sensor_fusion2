from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Nav1_gyro(ViewBase):
    name = "nav1_gyro"
    default_format = "nav1_gyro"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(8, 5), dpi=100)

        self.ax_gyro = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = [s.seq for s in self.samples]

        gx_values = self.get_series("gx")
        gy_values = self.get_series("gy")
        gz_values = self.get_series("gz")

        self.ax_gyro.clear()

        self.ax_gyro.plot(xs, gx_values, label="gx")
        self.ax_gyro.plot(xs, gy_values, label="gy")
        self.ax_gyro.plot(xs, gz_values, label="gz")
        self.ax_gyro.set_title("gyroscope")
        self.ax_gyro.set_xlabel("Seq")
        self.ax_gyro.set_ylabel("gyroscope")
        self.ax_gyro.legend(loc="upper right")
        self.ax_gyro.grid(True)

        self.fig.tight_layout()
        self.canvas.draw_idle()

