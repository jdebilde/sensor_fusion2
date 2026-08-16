# Useful Commands

This document lists useful Erlang commands that were used during the development, testing, and debugging of the `sensor_fusion` system.

These commands should be executed in an Erlang shell connected to the application—for example, after launching the system locally or via a remote shell on a GRiSP.

---

## 1. Restart Hera

```erlang
application:stop(hera).
application:start(hera).
```

These commands allow you to restart the `hera` application without restarting the entire board or the entire Erlang system.

This is useful when:

- a measurement needs to be restarted cleanly;
- Hera’s internal state appears to be stuck;
- you have just updated code related to the measurements;
- you want to stop the active measurements and then restart from a cleaner state.

In the `sensor_fusion` module, the `stop_all/0` function applies similar logic to the connected nodes: it remotely stops and then restarts the `hera` application.

---

## 2. Configure Measurement Arguments

```erlang
sensor_fusion:set_args(nav2).
sensor_fusion:set_args(uwb).
```

These commands prepare the necessary arguments before starting certain measurements.

### `sensor_fusion:set_args(nav2).`

This command calibrates the NAV Pmod using `nav2:calibrate()` and then stores the calibration in a shared ETS table.

This calibration is then retrieved when a navigation measurement is started.

### `sensor_fusion:set_args(uwb).`

This command prepares the UWB configuration, including the list of anchors and their positions.

Example configuration used in the code:

```erlang
Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}]}.
```

Each anchor is defined in the following format:

```erlang
{AnchorId, X, Y}
```

where `X` and `Y` are the anchor’s coordinates in meters.

---

## 3. Launch Measurements

```erlang
sensor_fusion:launch().
```

This command launches measurements appropriate for the current node type.

The module automatically determines whether the node is a `nav` node, a `uwb` node, or another node type based on its Erlang name.

### On a NAV Node

On a `nav`-type node, `sensor_fusion:launch()` retrieves the `nav2` calibration and starts the fusion measurement, for example:

```erlang
hera:start_measure(ekf6_nav2_uwb, {Calibration, Position}).
```

The initial position is defined in the code. It serves as the starting point for the filter.

### On a UWB node

On a `uwb`-type node, `sensor_fusion:launch()` first starts the UWB tag:

```erlang
uwb_tag:ensure_started().
```

Then it retrieves the UWB configuration and starts the measurement:

```erlang
hera:start_measure(uwb_measure, Config).
```

---

## 4. Reading Hera Data

```erlang
hera_data:get(uwb_measure, ‘sensor_fusion@uwb_3’).
hera_data:get(nav2, ‘sensor_fusion@nav_3’).
hera_data:get(bilateration, ‘sensor_fusion@uwb_3’).
```

These commands allow you to retrieve or display the data produced by a Hera measurement.

The general form is:

```erlang
hera_data:get(MeasureName, Node).
```

where:

- `MeasureName`: the name of the measurement, for example `uwb_measure`, `nav3`, `bilateration`;
- `Node`: the Erlang node that generates the measurement, for example `‘sensor_fusion@uwb_3’`.

These commands are useful for quickly verifying that a measurement is indeed generating data, independently of the Python viewer.

---

## 5. Updating Code on Remote Nodes

```erlang
sensor_fusion:update_code(sensor_fusion, nav2).
sensor_fusion:update_code(sensor_fusion, sensor_fusion).
sensor_fusion:update_code(sensor_fusion, uwb_tag).
sensor_fusion:update_code(sensor_fusion, uwb_anchor).
sensor_fusion:update_code(sensor_fusion, uwb_measure).
sensor_fusion:update_code(sensor_fusion, bilateration).
```

These commands allow you to compile a module locally and then copy it to the connected GRiSP nodes.

The general form is:

```erlang
sensor_fusion:update_code(Application, Module).
```

Example:

```erlang
sensor_fusion:update_code(sensor_fusion, uwb_measure).
```

This command:

1. compiles the module locally using `c:c(Module)`;
2. retrieves the generated `.beam` bytecode;
3. pings the expected nodes;
4. selects the currently connected nodes;
5. writes the new `.beam` file to the remote application;
6. forces the module to reload using `code:purge/1`, `code:delete/1`, and `code:load_file/1`.

This allows you to test changes without having to fully redeploy the SD cards.

After a major update, it is often helpful to restart Hera:

```erlang
application:stop(hera).
application:start(hera).
```

---

## 6. Read and Delete the Debug File

```erlang
{ok, Bin} = file:read_file(“debug.log”).
io:format(“~s”, [Bin]).
file:delete(“debug.log”).
```

These commands allow you to read the `debug.log` file, display its contents in the Erlang shell, and then delete it.

The `sensor_fusion` module also contains a utility function:

```erlang
sensor_fusion:print_debug().
```

This function reads `debug.log` and displays its contents using `io:format/2`.

---

## 7. Troubleshooting Connection Issues

### Display the Current Node

```erlang
node().
```

Displays the name of the current Erlang node.

Expected example:

```erlang
‘sensor_fusion@nav_3’
```

---

### Display Connected Nodes

```erlang
nodes().
```

Displays the list of Erlang nodes currently connected to the current node.

If a node does not appear in this list, it is not connected to the current Erlang cluster.

---

### Test the connection to a specific node

```erlang
net_adm:ping(‘sensor_fusion@nav_3’).
```

This command tests the connection to a specific Erlang node.

Possible results:

```erlang
pong
```

The node is accessible.

```erlang
pang
```

The node is not accessible.

In this case, you should verify:

- that both nodes are on the same network;
- that the hostnames are correct;
- that `/etc/hosts` is configured correctly;
- that the Erlang cookie is identical;
- that the application is running properly on the remote board.

---

### Connecting Expected Nodes

```erlang
sensor_fusion:connect_nodes().
```

This command pings all nodes expected by the system.

The list of expected nodes is defined in `sensor_fusion:target_nodes/0`.

Example of expected nodes:

```erlang
[
    ‘sensor_fusion@nav_3’,
    ‘sensor_fusion@uwb_1’,
    ‘sensor_fusion@uwb_2’,
    ‘sensor_fusion@uwb_3’
]
```

The function returns a list of results indicating whether each node is responding or not.

---

## 8. Typical Test Sequence

A typical sequence for restarting a UWB measurement might be:

```erlang
application:stop(hera).
application:start(hera).

sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

A typical sequence for restarting a NAV measurement might be:

```erlang
application:stop(hera).
application:start(hera).

sensor_fusion:set_args(nav2).
sensor_fusion:launch().
```

A typical sequence after modifying a module might be:

```erlang
sensor_fusion:stop_all().
sensor_fusion:connect_nodes().
sensor_fusion:update_code(sensor_fusion, uwb_measure).
```

---

## 9. Quick Summary

| Command | Role |
| --- | --- |
| `application:stop(hera).` | Stop Hera |
| `application:start(hera).` | Restart Hera |
| `sensor_fusion:stop_all().` | Stop and Restart Hera |
| `sensor_fusion:set_args(nav2).` | Calibrate and prepare NAV arguments |
| `sensor_fusion:set_args(uwb).` | Prepare UWB configuration |
| `sensor_fusion:launch().` | Launch measurements appropriate for the current node |
| `hera_data:get(Measure, Node). ` | Read data produced by a measurement |
| `sensor_fusion:update_code(sensor_fusion, Module).` | Update a module on remote nodes |
| `node().` | Display the current node |
| `nodes().` | Display connected nodes |
| `net_adm:ping(Node). ` | Test the connection to a node |
| `sensor_fusion:connect_nodes().` | Ping the expected nodes |
| `sensor_fusion:check_remote_module(Module).` | Check a module on remote nodes |
| `sensor_fusion:print_debug().` | Display `debug.log` |
