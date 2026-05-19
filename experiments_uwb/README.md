# Experiment 1
Facing to 0 deg

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

# Experiment 2
Facing to 90 deg

# experiment 3
Facing to 180 deg