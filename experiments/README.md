# Experiment 1
* The tag is facing the anchor (looking south)
* The anchor is facing the tag (looking north)

# Experiment 2
* The tag is facing the anchor (looking south)
* The anchor is facing right  (looking east)

# Experiment 3
* The tag is facing the anchor (looking south)
* The anchor is facing opposite (looking south)

## uwb_3 with uwb_1

```erlang
set_args(uwb) ->
    Anchor1 = {1, 0.0, 0.0},
    Anchors = [Anchor1],
    Current = 0,
    update_table({{uwb, node()}, { Anchors, Current} }).

launch(uwb) ->
    uwb_tag:ensure_started(),
    Anchors_and_current = ets:lookup_element(args, {uwb, node()}, 2),
    {ok,_} = hera:start_measure(uwb_measure, Anchors_and_current),
    ok;
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On uwb_1:
```
uwb_anchor:start(1).
```

## uwb_3 with uwb_2

```erlang
set_args(uwb) ->
    Anchor1 = {2, 0.0, 0.0},
    Anchors = [Anchor1],
    Current = 0,
    update_table({{uwb, node()}, { Anchors, Current} }).

launch(uwb) ->
    uwb_tag:ensure_started(),
    Anchors_and_current = ets:lookup_element(args, {uwb, node()}, 2),
    {ok,_} = hera:start_measure(uwb_measure, Anchors_and_current),
    ok;
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On uwb_2:
```
uwb_anchor:start(2).
```

## uwb_1 with uwb_2

```erlang
set_args(uwb) ->
    Anchor1 = {2, 0.0, 0.0},
    Anchors = [Anchor1],
    Current = 0,
    update_table({{uwb, node()}, { Anchors, Current} }).

launch(uwb) ->
    uwb_tag:ensure_started(),
    Anchors_and_current = ets:lookup_element(args, {uwb, node()}, 2),
    {ok,_} = hera:start_measure(uwb_measure, Anchors_and_current),
    ok;
```

On uwb_1:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On uwb_2:
```
uwb_anchor:start(2).
```

These files contains results form `uwb_measure` so **distance in cm to anchor i**.

# Experiment 4
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% bilateration
    Config = {{1, 4.70, 0.05}, {2, 0.10, 0.05}, {0.9, 3.15}},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(bilateration, Config)~n"),
    {ok,_} = hera:start_measure(bilateration, Config),
    ok;
```

# Experiment 5 to 10
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% uwb_measure
    Config = {[{1, 0.0, 0.0}]},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(uwb_measure, Config)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Config),
    ok;
```

# Experiment 12
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On nav_3:
```
sensor_fusion:set_args(nav2).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% uwb_measure
    Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}]},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(uwb_measure, Config)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Config),
    ok;

launch(nav) ->
    io:format("launch(nav)~n"),
    Calibration = ets:lookup_element(args, {nav2, node()}, 2),
    io:format("Calibration:~n~p~n", [Calibration]),

    % Position = {0.9, 3.15}, % rec with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    % Position = {1.7, 4.95}, % rec with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {1.2, 2.78}, % circle with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    Position = {1.2, 2.90}, % eight with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    io:format("Position:~n~p~n", [Position]),
    io:format("hera:start_measure(ekf4_nav2_uwb, {Calibration, Position})~n"),
    {ok,_} = hera:start_measure(ekf4_nav2_uwb, {Calibration, Position}),
    ok;
```

# Experiment 13
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On nav_3:
```
sensor_fusion:set_args(nav2).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% uwb_measure
    Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}]},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(uwb_measure, Config)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Config),
    ok;

launch(nav) ->
    io:format("launch(nav)~n"),
    Calibration = ets:lookup_element(args, {nav2, node()}, 2),
    io:format("Calibration:~n~p~n", [Calibration]),

    % Position = {0.9, 3.15}, % rec with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    % Position = {1.2, 2.78}, % circle with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    Position = {1.2, 2.90}, % eight with [{1, 2.30, 0.05}, {2, 0.10, 0.05}]
    io:format("Position:~n~p~n", [Position]),
    io:format("hera:start_measure(ekf6_nav2_uwb, {Calibration, Position})~n"),
    {ok,_} = hera:start_measure(ekf6_nav2_uwb, {Calibration, Position}),
    ok;
```

# Experiment 14
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On nav_3:
```
sensor_fusion:set_args(nav2).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% uwb_measure
    Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}]},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(uwb_measure, Config)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Config),
    ok;

launch(nav) ->
    io:format("launch(nav)~n"),
    Calibration = ets:lookup_element(args, {nav2, node()}, 2),
    io:format("Calibration:~n~p~n", [Calibration]),

    Position = {1.7, 3.15}, % rec with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {1.7, 4.95}, % rec with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {2.0, 2.78}, % circle with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {2.0, 2.90}, % eight with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    io:format("Position:~n~p~n", [Position]),
    io:format("hera:start_measure(ekf4_nav2_uwb, {Calibration, Position})~n"),
    {ok,_} = hera:start_measure(ekf4_nav2_uwb, {Calibration, Position}),
    ok;
```

# Experiment 15
On uwb_1:
```
uwb_anchor:start(1).
```

On uwb_2:
```
uwb_anchor:start(2).
```

On uwb_3:
```
sensor_fusion:set_args(uwb).
sensor_fusion:launch().
```

On nav_3:
```
sensor_fusion:set_args(nav2).
sensor_fusion:launch().
```

in `sensor_fusion.erl`
```erlang
set_args(uwb) ->
    %% uwb_measure
    Config = {[{1, 2.30, 0.05}, {2, 0.10, 0.05}]},
    update_table({{uwb, node()}, Config}).

launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),

    Config = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("Config:~n~p~n", [Config]),

    io:format("hera:start_measure(uwb_measure, Config)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Config),
    ok;

launch(nav) ->
    io:format("launch(nav)~n"),
    Calibration = ets:lookup_element(args, {nav2, node()}, 2),
    io:format("Calibration:~n~p~n", [Calibration]),

    Position = {1.7, 3.15}, % rec with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {1.7, 4.95}, % rec with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {2.0, 2.78}, % circle with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    % Position = {2.0, 2.90}, % eight with [{1, 4.70, 0.05}, {2, 0.10, 0.05}]
    io:format("Position:~n~p~n", [Position]),
    io:format("hera:start_measure(ekf6_nav2_uwb, {Calibration, Position})~n"),
    {ok,_} = hera:start_measure(ekf6_nav2_uwb, {Calibration, Position}),
    ok;
```