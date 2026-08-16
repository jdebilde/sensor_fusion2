Repository of the master thesis ["Sensor Fusion for High-Precision UWB Localization with GRISP"](thesis.pdf) by Jean-Jacques Debilde, David Alonso Alonso.
This repository is based on the work of successive master thesis:
-  from Julien Bastin and Guillaume Neirinckx : https://github.com/bastinjul/sensor_fusion
- from Sébastien Kalbusch and Vincent Verpoten : https://github.com/sebkm/sensor_fusion
- from Lucas Nélis : https://github.com/lunelis/sensor_fusion
- from Laurent Gwendal : https://github.com/GwendalLaurent/pmod_uwb and https://github.com/grisp/grisp/tree/gl/UWB

# User manual

## Required hardware
To use our system you need:
- 1 computer
- 1 wifi access point (a smartphone is enough)
- 4 or more GRiSP2 (with sd card)
- 1 or more PmodNAV
- 3 or more PmodUWB

You can find all GRiSP related hardware at https://www.grisp.org/shop/.


## Required software
To use our system you need to install on your computer:
- Erlang/OTP 25.0
- rebar3 3.18.0
- rebar3_hex 
- rebar3_grisp 
- Python3, python3-tk, python3-pil, python3-pil.imagetk (only for the visualization tool)

We advise you to work on GNU/Linux and to follow this tutorial https://github.com/grisp/grisp/wiki in case of problems.
First, you can install [Erlang/OTP](https://www.erlang.org/patches/otp-25.0).
Then, you can install [rebar3](https://github.com/erlang/rebar3/releases/tag/3.18.0) and follow this tutorial https://github.com/erlang/rebar3/#getting-started.
When this is done, you should specify the plugins in ~/.config/rebar3/rebar.config:

```erlang
{plugins, [
	{rebar3_hex},
    {rebar3_grisp}
]}.
```

and run the following command to update the plugins and verify if they are correctly installed:
```bash
rebar3 update && rebar3 plugins list
```

## Configuration files
### Network configuration
To connect the GRiSP boards via existing wifi network, you first need put the information relative to your network in [wpa_supplicant.conf](grisp/grisp2/common/deploy/files/wpa_supplicant.conf).
Then, you must indicate the IP addresses and hostnames in [erl_inetrc](grisp/grisp2/common/deploy/files/erl_inetrc).
For example:

```erlang
{host, {172,20,10,3}, ["hostname_computer"]}. % computer node
{host, {172,20,10,4}, ["nav_1"]}.
{host, {172,20,10,5}, ["nav_2"]}. % if necessary
{host, {172,20,10,6}, ["nav_3"]}. % if necessary
{host, {172,20,10,7}, ["uwb_1"]}.
{host, {172,20,10,8}, ["uwb_2"]}.
{host, {172,20,10,9}, ["uwb_3"]}.
```

To find the IP of a board, you can perform a network scan or follow the tutorial from grisp.
Finally, you should write this information on your computer in [/etc/hosts]().
The format is not the same.
Here is an example:
```
127.0.1.1		hostname_computer
172.20.10.4     nav_1
172.20.10.5     nav_2                                                               
172.20.10.6     nav_3                                                               
172.20.10.7     uwb_1                                                               
172.20.10.8     uwb_2                                                               
172.20.10.9     uwb_3 
```
If you wish to use our system as is then you must follow the same hostnames as shown in the example (More on that later).

### Other configurations
[computer.config.src](config/computer.config.src) is used for the computer node.
The log_data variable must be set to true if you wish to receive the data collected on the network.
```erlang
{hera, [
    {log_data, true}
]}
```
The data will be written in **csv** format in [measures/](./measures/).
The file names follow a specific nomenclature: *measureName_sensor_fusion@hostname.csv*.

[sys.config](config/sys.config) is used for the GRiSP nodes.
An error logger is setup to generate a report in LOGS/ERROR.1 on the sd card.
If you suspect something has gone wrong with the system you should look there, but be aware that the information might be outdated as these files are never removed.

[vm.args](config/vm.args) must contain:
```
## Name of the node
-sname sensor_fusion

## Cookie for distributed erlang
-setcookie MyCookie
```

Finally, [rebar.config](rebar.config) contains all the information to build and deploy the system.
The only part you should modify is the path to the sd card on which you want to deploy the system.
In this example, the sd card is names "GRISP".
``` erlang
{deploy , [
	{pre_script, "rm -rf /media/hostname/GRISP/*"},
	{destination, "/media/hostname/GRISP"},
	{post_script, "umount /media/hostname/GRISP"}
]}
```

If you want to have more in-depth information about configuration files here are a few useful links:
- https://github.com/grisp/grisp/wiki
- https://github.com/grisp/rebar3_grisp
- https://github.com/erlang/rebar3
- http://erlang.org/doc/man/config.html
- http://erlang.org/documentation/doc-5.9/doc/design_principles/distributed_applications.html

## Using Numerl (Custom OTP)
In order to use the Numerl NIF and the driver for PmodUWB, you must first compile a custom version of OTP. This can be done in multiple way (see https://github.com/grisp/grisp/wiki/Building-the-VM-from-source). The fastest way is to use the docker method. To do so first install docker. First, add the following line to the grisp/build/toolchain section of [rebar.config](rebar.config)
``` erlang
{deps, [
	{hera, {git , "https://github.com/lunelis/hera" , {branch , "main"}}},
    ...
    {grisp, {git, "https://github.com/grisp/grisp", {branch, "gl/UWB"}}},
	...
]}.

{grisp, [
	{build, [
		{toolchain, [
			{docker, "grisp/grisp2-rtems-toolchain"}
		]}
	]}
]}
```

Then, run the following command.
```bash
rebar3 grisp build --docker 
```
This will generate a _grisp folder containing the VM running on the GRiSP. Using NIFs can only be done by recompiling the whole OTP version due to GRiSP's limitation (https://github.com/grisp/grisp/wiki/NIF-Support).

## Deployment
The first thing to do is to format each sd card as **fat32**.
We also suggest to name it "GRISP".
The easiest way to achieve that is to use a partitioning tool like "KDE Partition Manager" or similar.
You only need to this once.

To ease the process we created a make file that we use as a command shortcut.
You can deploy the software on each sd card with the `make deploy-hostname` command where hostname is the name of the GRiSP2 board.
This will take some time.
For example: 
```bash
make deploy-nav_2
```

After that, you can plug the sd cards in their respective GRiSP2 boards.
Then, you should plug the Pmod sensors:

| Pmod      | Slot  |
| :---:     | :---: |
| NAV       | SPI2  |
| UWB       | SPI2  |


Finally, you can connect the board to the battery.
During the boot phase you should see one green LED.
After ~1 min you should see two red LEDs or two green LEDs (see later).
In case of problems we advise you to connect the GRiSP-base by serial https://github.com/grisp/grisp/wiki/Connecting-over-Serial.
You can use `make screen` once the cable is plugged in.

In parallel you can start the application on your computer.
You can either have a clean start with:
```bash
make local_release && make run_local
```
or start in developement mode with:
```bash
make shell
```
We advise you to start in developement mode.
Note that if you use the release the measures folder will be created in the release root directory.
After a few seconds a message will be displayed telling you that the application is booted.
You might need to press enter to get the shell prompt.

## Launching the system

### Calibration
Certain sensors require a calibration in order to be used.
If the information is not present on the system, you should see two red LEDs.
On the other hand, if the information is present, you should see two green LEDs.
Note that in case of restart, as long as there is at least one node staying alive, the information will remain available.

The calibration routine depends on the node hostname as well as the type of measurements that you wish to perform.
The first step we suggest is to open a remote shell.
If you know what you are doing you can also perform the calibrations with the `rpc` module.
When the calibration is done you can close the remote shell.

#### Calibration of a nav node
First, you should open a remote shell to the node.
For instance:
```bash
make remote-nav_3
```
Then you must call:
```erlang
sensor_fusion:set_args(nav2).
sensor_fusion:laucnh().
```

You can look into [sensor_fusion.erl](src/sensor_fusion.erl) to see how it works.
```erlang
sensor_fusion:set_args(nav2).

Calibration = ets:lookup_element(args, {nav2, node()}, 2).

{ok,_} = hera:start_measure(nav2, Calibration).

{ok,_} = hera:start_measure(ekf4_nav2_uwb, Calibration).

hera_data:get(nav2, sensor_fusion@nav_3).

hera_data:get(ekf4_nav2_uwb, sensor_fusion@nav_3).

sensor_fusion:stop_all().
```

### Launching the anchors
First, you should open a remote shell to the node.
For instance:
```bash
make remote-uwb_1
```
Then you must call:
```erlang
uwb_anchor:start(1).
```

In our code, we used ID (here 1) to identify each uwb. A better alternative
could be to use mac addresses in the future.

### Launching the tags
First, you should open a remote shell to the node.
For instance:
```bash
make remote-uwb_3
```
Then you must call:
```erlang
Config = {[{1, 4.70, 0.05}, {2, 0.10, 0.05}]},
{ok,_} = hera:start_measure(uwb_measure, Config),

Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}], {0.9, 3.15}},
{ok,_} = hera:start_measure(bilateration, Config),
```
You can define in a list each anchor with (ID, X, Y), `uwb_measure` will measure
the distance to each anchor in a loop. For the bilateration, you need to give
a starting position that helps choosing between the two possible solutions.

### Launching the measurements
You can launch the whole cluster from any node with:
```erlang
sensor_fusion:launch_all().
```
Alternatively you can launch a single node from the remote shell with:
```erlang
sensor_fusion:launch().
```
If the LEDs switch to green, you can consider the system to be launched.
If they switch to red (or remain red) it means the calibration data was not found.

If you wish to stop the measurements on the whole cluster you can use:
```erlang
sensor_fusion:stop_all().
```

---

### LiveView
You can visualize the measurements in soft real time with the LiveView tool.
To start it just do:
```bash
make liveView
```
For more information, you can check our small manual about the [viewer](/markdown/viewer.md).

---

### Experiements

In this [document](experiments/README.md), we describe the commands we used and provide explanations of the experiments we conducted.

---

### Useful commands

In this [document](/markdown/HOW_TO.md), we provide some useful commands that we used during development. This can serve as a quick reference when you forget a command.

