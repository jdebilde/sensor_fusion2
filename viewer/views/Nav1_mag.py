from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Nav1_mag(ViewBase):
    name = "nav1_mag"
    default_format = "nav1_mag"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(8, 5), dpi=100)

        self.ax_mag = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = [s.seq for s in self.samples]

        mx_values = self.get_series("mx")
        my_values = self.get_series("my")
        mz_values = self.get_series("mz")

        self.ax_mag.clear()

        self.ax_mag.plot(xs, mx_values, label="mx")
        self.ax_mag.plot(xs, my_values, label="my")
        self.ax_mag.plot(xs, mz_values, label="mz")
        self.ax_mag.set_title("magnetic field")
        self.ax_mag.set_xlabel("Seq")
        self.ax_mag.set_ylabel("magnetic field")
        self.ax_mag.legend(loc="upper right")
        self.ax_mag.grid(True)

        self.fig.tight_layout()
        self.canvas.draw_idle()

