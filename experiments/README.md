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
Test uwb alone!!

# experiment 5
We use `uwb_nav_ekf` to measure different static positions and try moving along 
the table. We have here postion on **position x, position y, velocity x, velocity y**.