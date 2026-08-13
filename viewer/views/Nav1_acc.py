from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Nav1_acc(ViewBase):
    name = "nav1_acc"
    default_format = "nav1_acc"

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

        ax_values = self.get_series("ax")
        ay_values = self.get_series("ay")
        az_values = self.get_series("az")

        self.ax_acc.clear()

        self.ax_acc.plot(xs, ax_values, label="ax")
        self.ax_acc.plot(xs, ay_values, label="ay")
        self.ax_acc.plot(xs, az_values, label="az")
        self.ax_acc.set_title("Acceleration")
        self.ax_acc.set_xlabel("Seq")
        self.ax_acc.set_ylabel("Acceleration")
        self.ax_acc.legend(loc="upper right")
        self.ax_acc.grid(True)

        self.fig.tight_layout()
        self.canvas.draw_idle()

