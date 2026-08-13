import os
import matplotlib.pyplot as plt


class ViewBase:
    name = "base"
    default_format = None

    def __init__(self, parent):
        self.parent = parent
        self.samples = []

    def add_sample(self, sample):
        self.samples.append(sample)

        if len(self.samples) > 1000:
            self.samples = self.samples[-1000:]

    def add_samples(self, samples):
        self.samples.extend(samples)

    def redraw(self):
        raise NotImplementedError

    def export_subplots(self, output_dir="exports", prefix="plot"):
        os.makedirs(output_dir, exist_ok=True)

        # Make sure the current figure is rendered
        self.canvas.draw()

        for i, old_ax in enumerate(self.fig.axes, start=1):
            new_fig, new_ax = plt.subplots(figsize=(8, 5), dpi=150)

            # Copy lines
            for line in old_ax.get_lines():
                new_ax.plot(
                    line.get_xdata(),
                    line.get_ydata(),
                    label=line.get_label(),
                    marker=line.get_marker(),
                    markersize=line.get_markersize(),
                    linestyle=line.get_linestyle(),
                    linewidth=line.get_linewidth(),
                )

            # Copy scatter plots / collections
            for collection in old_ax.collections:
                offsets = collection.get_offsets()
                if len(offsets) > 0:
                    new_ax.scatter(
                        offsets[:, 0],
                        offsets[:, 1],
                        s=collection.get_sizes(),
                        marker="o",
                        label=collection.get_label(),
                    )

            # Copy rectangles / patches, useful for tables
            for patch in old_ax.patches:
                from matplotlib.patches import Rectangle

                if isinstance(patch, Rectangle):
                    x, y = patch.get_xy()
                    new_patch = Rectangle(
                        (x, y),
                        patch.get_width(),
                        patch.get_height(),
                        fill=patch.get_fill(),
                        linewidth=patch.get_linewidth(),
                        edgecolor=patch.get_edgecolor(),
                        facecolor=patch.get_facecolor(),
                    )
                    new_ax.add_patch(new_patch)

            # Copy text annotations
            for text in old_ax.texts:
                x, y = text.get_position()
                new_ax.text(
                    x,
                    y,
                    text.get_text(),
                    ha=text.get_ha(),
                    va=text.get_va(),
                    fontsize=text.get_fontsize(),
                )

            # Copy labels/title/grid/limits
            new_ax.set_title(old_ax.get_title())
            new_ax.set_xlabel(old_ax.get_xlabel())
            new_ax.set_ylabel(old_ax.get_ylabel())
            new_ax.set_xlim(old_ax.get_xlim())
            new_ax.set_ylim(old_ax.get_ylim())

            if old_ax.get_aspect() == "equal" or old_ax.get_aspect() == 1.0:
                new_ax.set_aspect("equal", adjustable="box")

            new_ax.grid(True)

            # Add legend only if useful labels exist
            handles, labels = new_ax.get_legend_handles_labels()
            labels = [label for label in labels if label and not label.startswith("_")]
            if labels:
                new_ax.legend(loc="upper right")

            filename = os.path.join(output_dir, f"{prefix}_{i}.png")

            new_fig.tight_layout()
            new_fig.savefig(filename, dpi=200, bbox_inches="tight", pad_inches=0.25)
            plt.close(new_fig)

            print(f"Exported {filename}")