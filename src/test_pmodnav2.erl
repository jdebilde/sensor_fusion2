-module(test_pmodnav2).

-export([
    calibrate/0,
    collect_acc_gyro/2,
    loop_print_acc_gyro/2,
    print_samples/1
]).

-define(G, 9.80665).

-record(cal, {
    gyro = {0.0, 0.0, 0.0}
}).

-record(sample, {
    t_us,
    dt_s,
    acc_g,
    acc_ms2,
    gyro,
    gyro_corrected
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("nav3: Place the pmod_nav flat and still!~n"),
    %% NOTE: if your driver requires `acc` for gyro registers, keep acc here.
    %% Otherwise try replacing acc with gyro.
    [Gx, Gy, Gz] = calibrate(acc, [out_x_g, out_y_g, out_z_g], 2000),
    #cal{gyro = {Gx, Gy, Gz}}.


%% Main function: collect N samples in RAM, then print stats once.
collect_acc_gyro(N, C = #cal{}) when is_integer(N), N > 0 ->
    StartT = erlang:monotonic_time(microsecond),
    Samples = collect_loop(N, C, StartT, []),
    print_stats(Samples),
    Samples.


%% Kept for compatibility with your old command name.
loop_print_acc_gyro(N, C) ->
    collect_acc_gyro(N, C).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Collection loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

collect_loop(0, _C, _PrevT, Acc) ->
    lists:reverse(Acc);

collect_loop(N, C, PrevT, Acc) ->
    Now = erlang:monotonic_time(microsecond),
    Dt = (Now - PrevT) / 1000000.0,

    Sample = read_sample(C, Now, Dt),

    collect_loop(N - 1, C, Now, [Sample | Acc]).


read_sample(#cal{gyro = {GBx, GBy, GBz}}, Timestamp, Dt) ->
    [Ax, Ay, Az, Gx, Gy, Gz] = pmod_nav:read(acc, [
        out_x_xl, out_y_xl, out_z_xl,
        out_x_g,  out_y_g,  out_z_g
    ]),

    AccMs2 = {Ax * ?G, Ay * ?G, Az * ?G},
    Gyro = {Gx, Gy, Gz},
    GyroCorrected = {Gx - GBx, Gy - GBy, Gz - GBz},

    #sample{
        t_us = Timestamp,
        dt_s = Dt,
        acc_g = {Ax, Ay, Az},
        acc_ms2 = AccMs2,
        gyro = Gyro,
        gyro_corrected = GyroCorrected
    }.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Stats
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

print_stats(Samples) ->
    N = length(Samples),

    AccMs2 = [S#sample.acc_ms2 || S <- Samples],
    GyroCorrected = [S#sample.gyro_corrected || S <- Samples],
    Dts = [S#sample.dt_s || S <- Samples],

    {AccX, AccY, AccZ} = unzip3(AccMs2),
    {Gx, Gy, Gz} = unzip3(GyroCorrected),

    io:format("Number of samples: ~p~n", [N]),

    print_axis("acc_x", AccX),
    print_axis("acc_y", AccY),
    print_axis("acc_z", AccZ),
    io:format("~n"),

    print_axis("gyro_x_corrected", Gx),
    print_axis("gyro_y_corrected", Gy),
    print_axis("gyro_z_corrected", Gz),
    io:format("~n"),

    print_axis("dt", Dts),

    TotalDt = lists:sum(Dts),
    io:format("total dt ~p s~n", [TotalDt]),

    ok.


print_axis(Name, Values) ->
    Min = lists:min(Values),
    Mean = mean(Values),
    Max = lists:max(Values),
    io:format("~s min ~p mean ~p max ~p~n", [Name, Min, Mean, Max]).


mean(Values) ->
    lists:sum(Values) / length(Values).


unzip3(Tuples) ->
    Xs = [X || {X, _Y, _Z} <- Tuples],
    Ys = [Y || {_X, Y, _Z} <- Tuples],
    Zs = [Z || {_X, _Y, Z} <- Tuples],
    {Xs, Ys, Zs}.

print_samples(Samples) ->
    lists:foreach(fun print_sample/1, Samples).


print_sample(#sample{
    t_us = Timestamp,
    acc_g = {Ax, Ay, Az},
    acc_ms2 = {AxMs2, AyMs2, AzMs2},
    gyro = {Gx, Gy, Gz},
    gyro_corrected = {CGx, CGy, CGz}
}) ->
    io:format(
        "{\"timestamp\":~p,"
        "\"acc_g\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"acc_ms2\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"gyro\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"gyro_corrected\":{\"x\":~p,\"y\":~p,\"z\":~p}}~n",
        [
            Timestamp,
            Ax, Ay, Az,
            AxMs2, AyMs2, AzMs2,
            Gx, Gy, Gz,
            CGx, CGy, CGz
        ]
    ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate(Comp, Registers, N) ->
    Data = [
        list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1, N)
    ],
    {X, Y, Z} = unzip3(Data),
    [
        lists:sum(X) / N,
        lists:sum(Y) / N,
        lists:sum(Z) / N
    ].