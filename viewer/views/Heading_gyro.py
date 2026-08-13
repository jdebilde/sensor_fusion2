from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Heading_gyro(ViewBase):
    name = "heading_gyro"
    default_format = "heading_gyro"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(8, 5), dpi=100)

        self.ax_acc = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = [s.seq for s in self.samples]

        roll_values = self.get_series("roll")
        pitch_values = self.get_series("pitch")
        yaw_values = self.get_series("yaw")

        self.ax_acc.clear()

        self.ax_acc.plot(xs, roll_values, label="roll")
        self.ax_acc.plot(xs, pitch_values, label="pitch")
        self.ax_acc.plot(xs, yaw_values, label="yaw")
        self.ax_acc.set_title("Roll, Pitch and Yaw")
        self.ax_acc.set_xlabel("Seq")
        self.ax_acc.set_ylabel("Roll, Pitch and Yaw")
        self.ax_acc.legend(loc="upper right")
        self.ax_acc.grid(True)

        self.fig.tight_layout()
        self.canvas.draw_idle()

