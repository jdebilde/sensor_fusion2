from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Ekf_v1(ViewBase):
    name = "ekf_v1"
    default_format = "ekf_v1"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(11, 8), dpi=100)

        self.ax_angles = self.fig.add_subplot(321)
        self.ax_gyro = self.fig.add_subplot(322)
        self.ax_acc = self.fig.add_subplot(323)
        self.ax_grav = self.fig.add_subplot(324)
        self.ax_lin_acc = self.fig.add_subplot(325)
        self.ax_world_acc = self.fig.add_subplot(326)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def get_x_axis(self):
        # Tu peux remplacer par nav_timestamp si tu l’as dans ce format.
        return [s.seq for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = self.get_x_axis()

        roll_values = self.get_series("roll")
        pitch_values = self.get_series("pitch")
        yaw_values = self.get_series("yaw")

        gyro_x_values = self.get_series("gyro_x")
        gyro_y_values = self.get_series("gyro_y")
        gyro_z_values = self.get_series("gyro_z")

        acc_x_values = self.get_series("acc_x")
        acc_y_values = self.get_series("acc_y")
        acc_z_values = self.get_series("acc_z")

        grav_res_x_values = self.get_series("grav_res_x")
        grav_res_y_values = self.get_series("grav_res_y")
        grav_res_z_values = self.get_series("grav_res_z")

        lin_acc_x_values = self.get_series("lin_acc_x")
        lin_acc_y_values = self.get_series("lin_acc_y")
        lin_acc_z_values = self.get_series("lin_acc_z")

        world_acc_x_values = self.get_series("world_acc_x")
        world_acc_y_values = self.get_series("world_acc_y")

        self.ax_angles.clear()
        self.ax_gyro.clear()
        self.ax_acc.clear()
        self.ax_grav.clear()
        self.ax_lin_acc.clear()
        self.ax_world_acc.clear()

        # 1) Roll / pitch / yaw
        self.ax_angles.plot(xs, roll_values, label="roll")
        self.ax_angles.plot(xs, pitch_values, label="pitch")
        self.ax_angles.plot(xs, yaw_values, label="yaw")
        self.ax_angles.set_title("Orientation")
        self.ax_angles.set_xlabel("Seq")
        self.ax_angles.set_ylabel("deg")
        self.ax_angles.legend(loc="upper right")
        self.ax_angles.grid(True)

        # 2) Gyroscope brut
        self.ax_gyro.plot(xs, gyro_x_values, label="gyro_x")
        self.ax_gyro.plot(xs, gyro_y_values, label="gyro_y")
        self.ax_gyro.plot(xs, gyro_z_values, label="gyro_z")
        self.ax_gyro.set_title("Gyroscope brut")
        self.ax_gyro.set_xlabel("Seq")
        self.ax_gyro.set_ylabel("gyro")
        self.ax_gyro.legend(loc="upper right")
        self.ax_gyro.grid(True)

        # 3) Accélération brute
        self.ax_acc.plot(xs, acc_x_values, label="acc_x")
        self.ax_acc.plot(xs, acc_y_values, label="acc_y")
        self.ax_acc.plot(xs, acc_z_values, label="acc_z")
        self.ax_acc.set_title("Acceleration brute")
        self.ax_acc.set_xlabel("Seq")
        self.ax_acc.set_ylabel("acc")
        self.ax_acc.legend(loc="upper right")
        self.ax_acc.grid(True)

        # 4) Résidu / estimation gravité
        self.ax_grav.plot(xs, grav_res_x_values, label="grav_res_x")
        self.ax_grav.plot(xs, grav_res_y_values, label="grav_res_y")
        self.ax_grav.plot(xs, grav_res_z_values, label="grav_res_z")
        self.ax_grav.set_title("Residu de gravite")
        self.ax_grav.set_xlabel("Seq")
        self.ax_grav.set_ylabel("grav")
        self.ax_grav.legend(loc="upper right")
        self.ax_grav.grid(True)

        # 5) Accélération linéaire sans gravité
        self.ax_lin_acc.plot(xs, lin_acc_x_values, label="lin_acc_x")
        self.ax_lin_acc.plot(xs, lin_acc_y_values, label="lin_acc_y")
        self.ax_lin_acc.plot(xs, lin_acc_z_values, label="lin_acc_z")
        self.ax_lin_acc.set_title("Acceleration lineaire")
        self.ax_lin_acc.set_xlabel("Seq")
        self.ax_lin_acc.set_ylabel("acc lin")
        self.ax_lin_acc.legend(loc="upper right")
        self.ax_lin_acc.grid(True)

        # 6) Accélération finale monde X/Y
        self.ax_world_acc.plot(xs, world_acc_x_values, label="world_acc_x")
        self.ax_world_acc.plot(xs, world_acc_y_values, label="world_acc_y")
        self.ax_world_acc.set_title("Acceleration repere monde")
        self.ax_world_acc.set_xlabel("Seq")
        self.ax_world_acc.set_ylabel("acc monde")
        self.ax_world_acc.legend(loc="upper right")
        self.ax_world_acc.grid(True)

        self.fig.tight_layout()
        self.canvas.draw_idle()