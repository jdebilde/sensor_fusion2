from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, Circle
import tkinter as tk


class Position_stevin(ViewBase):
    name = "position_stevin"
    default_format = "Ekf6_nav2_uwb"

    # Coordonnées en mètres
    ANCHORS = [
        (1, 2.30, 0.05),
        (2, 0.10, 0.05),
    ]

    # Tables en mètres: (x, y, width, height)
    TABLES = [
        (0.0, 0.0, 0.8, 1.6),
        (0.8, 0.0, 0.8, 1.6),
        (1.6, 0.0, 0.8, 1.6),
        (0.8, 1.6, 0.8, 1.6),
    ]

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(9, 7), dpi=100)
        self.ax_pos = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        px_values = self.get_series("px")
        py_values = self.get_series("py")

        px_cm = [px * 100.0 if px is not None else None for px in px_values]
        py_cm = [py * 100.0 if py is not None else None for py in py_values]

        self.ax_pos.clear()

        # Tables et ancres d'abord, pour rester en arrière-plan
        self.draw_tables()
        self.draw_anchors()
        self.draw_real_trajectory(1) 

        # Position 2D
        self.ax_pos.plot(px_cm, py_cm, marker="o", markersize=2, label="trajectory")

        self.ax_pos.set_title("Position 2D")
        self.ax_pos.set_xlabel("px [cm]")
        self.ax_pos.set_ylabel("py [cm]")
        self.ax_pos.grid(True)
        self.ax_pos.axis("equal")

        # Dernier point, aussi en cm
        if px_cm and py_cm:
            last_px = px_cm[-1]
            last_py = py_cm[-1]

            if last_px is not None and last_py is not None:
                self.ax_pos.scatter([last_px], [last_py], s=50, label="current")

        self.set_min_axis_range(
            self.ax_pos,
            px_cm,
            py_cm,
            min_range_cm=50.0,
            extra_points_cm=self.static_points_cm()
        )

        self.ax_pos.legend(loc="upper right")

        self.fig.tight_layout()
        self.canvas.draw_idle()

    def draw_anchors(self):
        for anchor_id, x_m, y_m in self.ANCHORS:
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0

            self.ax_pos.scatter([x_cm], [y_cm], marker="^", s=80)
            self.ax_pos.text(
                x_cm,
                y_cm + 5,
                f"A{anchor_id}",
                ha="center",
                va="bottom"
            )

    def draw_tables(self):
        for index, (x_m, y_m, width_m, height_m) in enumerate(self.TABLES, start=1):
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0
            width_cm = width_m * 100.0
            height_cm = height_m * 100.0

            rect = Rectangle(
                (x_cm, y_cm),
                width_cm,
                height_cm,
                fill=False,
                linewidth=2
            )

            self.ax_pos.add_patch(rect)

            self.ax_pos.text(
                x_cm + width_cm / 2,
                y_cm + height_cm / 2,
                f"T{index}",
                ha="center",
                va="center"
            )
    
    def draw_real_trajectory(self, index=0):
        """
        This function allows you to plot the actual trajectory
        """
        if index == 0:
            return
        
        elif index == 1:
            x_m, y_m, width_m, height_m = 1 * 0.8 + 0.115, 1 * 1.6 + 0.15, 0.575, 1.40
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0
            width_cm = width_m * 100.0
            height_cm = height_m * 100.0

            rect = Rectangle(
                (x_cm, y_cm), width_cm, height_cm, fill=False, linewidth=2,
                linestyle='dotted', color='red')

            self.ax_pos.add_patch(rect)
        
        elif index == 2:
            x_m, y_m = 1 * 0.8 + 0.4, 1 * 1.6 + 0.8
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0

            circle = Circle(xy=(x_cm, y_cm), radius=38, fill=False, 
                edgecolor='red', linewidth=2, linestyle='dotted')

            self.ax_pos.add_patch(circle)
        
        elif index == 3:
            x_m, y_m = 1 * 0.8 + 0.4, 1 * 1.6 + 0.8 - 0.25
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0

            circle = Circle(xy=(x_cm, y_cm), radius=25, fill=False, 
                edgecolor='red', linewidth=2, linestyle='dotted')

            self.ax_pos.add_patch(circle)
            x_m, y_m = 1 * 0.8 + 0.4, 1 * 1.6 + 0.8 + 0.25
            x_cm = x_m * 100.0
            y_cm = y_m * 100.0

            circle = Circle(xy=(x_cm, y_cm), radius=25, fill=False, 
                edgecolor='red', linewidth=2, linestyle='dotted')

            self.ax_pos.add_patch(circle)

    def static_points_cm(self):
        """
        Points fixes à inclure dans les limites du graphe :
        coins des tables + ancres.
        """
        points = []

        for _, x_m, y_m in self.ANCHORS:
            points.append((x_m * 100.0, y_m * 100.0))

        for x_m, y_m, width_m, height_m in self.TABLES:
            x0 = x_m * 100.0
            y0 = y_m * 100.0
            x1 = (x_m + width_m) * 100.0
            y1 = (y_m + height_m) * 100.0

            points.extend([
                (x0, y0),
                (x1, y0),
                (x0, y1),
                (x1, y1),
            ])

        return points

    @staticmethod
    def set_min_axis_range(ax, xs, ys, min_range_cm=50.0, extra_points_cm=None):
        valid_x = [x for x in xs if x is not None]
        valid_y = [y for y in ys if y is not None]

        if extra_points_cm:
            for x, y in extra_points_cm:
                valid_x.append(x)
                valid_y.append(y)

        if not valid_x or not valid_y:
            half = min_range_cm / 2.0
            ax.set_xlim(-half, half)
            ax.set_ylim(-half, half)
            return

        min_x, max_x = min(valid_x), max(valid_x)
        min_y, max_y = min(valid_y), max(valid_y)

        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0

        range_x = max_x - min_x
        range_y = max_y - min_y

        axis_range = max(range_x, range_y, min_range_cm)
        half = axis_range / 2.0

        margin = axis_range * 0.08

        ax.set_xlim(center_x - half - margin, center_x + half + margin)
        ax.set_ylim(center_y - half - margin, center_y + half + margin)