# How to use the viewer


### LiveView

You can visualize measurements with the LiveView tool. The viewer can either listen to Hera UDP packets in real time or read previously recorded CSV files.

To start the viewer, run:

```bash
make liveView
```

The interface contains the following fields:

* **Source**: choose between `Live UDP` and `CSV`.
* **PC IP**: IP address of the computer running the viewer. This is used to join the Hera multicast group.
* **Measure**: name of the Hera measure to display, for example `nav2`, `bilateration`, or `ekf6_nav2_uwb`.
* **Node**: Erlang node that produced the measure, for example `sensor_fusion@nav_3`.
* **Vue**: visualization to use.
* **CSV mode**: choose how a CSV file is displayed:

  * `replay`: display samples progressively, as if they were received live.
  * `full`: load the whole CSV file and display all samples directly.
* **Export PNG**: export the current view to PNG files in the `exports/` folder.

#### Reading live data

To read live data, first make sure that the GRiSP nodes and the local computer are on the same network and that the computer IP is correctly configured. For example:

```erlang
{host, {172,20,10,3}, ["hostname_computer"]}.
{host, {172,20,10,6}, ["nav_3"]}.
```

Then launch the measurements on the GRiSP nodes. For example:

```erlang
sensor_fusion:launch_all().
```

In the viewer:

1. Set **Source** to `Live UDP`.
2. Set **PC IP** to the IP address of the computer, for example `172.20.10.3`.
3. Select the desired **Measure**, for example `nav2`.
4. Set **Node** to the producer node, for example `sensor_fusion@nav_3`.
5. Select the desired **Vue**.
6. Click **Start**.

The viewer listens to the Hera multicast packets, decodes the Erlang binary term format, filters the packets according to the selected measure and node, and displays the matching samples.

#### Reading a CSV file

Hera can write received data in CSV format when `log_data` is enabled in the computer configuration:

```erlang
{hera, [
    {log_data, true}
]}
```

The files are written in the `measures/` directory. Their names follow the pattern:

```text
measureName_sensor_fusion@hostname.csv
```

To read a CSV file in the viewer:

1. Set **Source** to `CSV`.
2. Click **Select CSV** and choose a file from `measures/`.
3. Select the corresponding **Measure** and **Node**.
4. Select the desired **Vue**.
5. Select the **CSV mode**:

   * `replay` to replay the file progressively.
   * `full` to display the entire file directly.
6. Click **Start**.

The CSV format expected by the viewer is:

```text
seq, heraTimestamp, values...
```

The first column is the Hera sequence number, the second column is the Hera timestamp, and the remaining columns are interpreted according to the selected format.

For example, for a `nav2` file, the values are interpreted as:

```text
nav_timestamp, dt, ax, ay, az, gx, gy, gz
```

#### Exporting figures

The current view can be exported with the **Export PNG** button. The exported files are written in:

```text
exports/
```

The filename prefix is based on the selected CSV filename when a CSV file is used.

### Adding a new view in LiveView

A view is a Python class located in the `views/` directory. Each view must inherit from `ViewBase`, define a name, define the default data format to use, and implement the `redraw()` method.

A minimal view looks like this:

```python
from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class MyView(ViewBase):
    name = "my_view"
    default_format = "my_format"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(9, 7), dpi=100)
        self.ax = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = [s.seq for s in self.samples]
        values = self.get_series("value")

        self.ax.clear()
        self.ax.plot(xs, values, label="value")
        self.ax.set_title("My view")
        self.ax.set_xlabel("Seq")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)
        self.ax.legend(loc="upper right")

        self.fig.tight_layout()
        self.canvas.draw_idle()
```

The data format must be added in `HeraSample.py`. A format maps the list of values received from Hera to named Python attributes:

```python
FORMATS = {
    "my_format": {
        "columns": [
            "timestamp",
            "value"
        ]
    }
}
```

For example, if Hera sends:

```erlang
{hera_data, my_measure, node(), Seq, [Timestamp, Value]}
```

then the view can access:

```python
sample.timestamp
sample.value
```

After creating the view, import it in `main.py`:

```python
from views.MyView import MyView
```

register it in the `VIEWS` dictionary:

```python
VIEWS = {
    MyView.name: MyView,
}
```

and add the hera_measure name for the measure in the `MEASUREMENTS` list
```python
MEASUREMENTS = [
    'nav1_acc',
]
```

The view will then appear in the **Vue** dropdown menu. When the view is selected, its `default_format` is automatically used to interpret the incoming UDP or CSV data.
