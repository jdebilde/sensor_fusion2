-module(test_pmodnav).

-export([
    calibrate/0,
    print_acc_gyro/1,
    loop_print_acc_gyro/2,
    print_acc/2,
    print_gyro/3,
    loop_print_acc_angle/2
]).

-define(G, 9.80665). % m/s^2
-define(LOOP_SLEEP_MS, 10).

% https://hexdocs.pm/grisp/2.9.0/pmod_nav.html#summary

-record(cal, {gyro}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("nav3 (gyro): Place the pmod_nav flat and still!~n"),
    % Gyro, degrees per second.
    % [Gx, Gy, Gz] = calibrate(acc, [out_x_g, out_y_g, out_z_g], 500),
    [Gx, Gy, Gz] = [0, 0, 0],
    io:format("calibrate: (~p, ~p, ~p)~n", [Gx, Gy, Gz]),
    #cal{gyro = {Gx, Gy, Gz}}.


print_acc_gyro(C = #cal{}) ->
    % Accelerometer, default unit: g
    % [out_x_xl, out_y_xl, out_z_xl]
    % Gyroscope, default unit: dps
    % [out_x_g, out_y_g, out_z_g]
    [Ax, Ay, Az, Gx, Gy, Gz] = pmod_nav:read(acc, [
        out_x_xl, out_y_xl, out_z_xl,
        out_x_g, out_y_g, out_z_g
    ]),
    Timestamp = erlang:monotonic_time(microsecond),
    print_acc_gyro_values(C, Timestamp, Ax, Ay, Az, Gx, Gy, Gz).


loop_print_acc_gyro(0, _C) ->
    ok;
loop_print_acc_gyro(N, C) when is_integer(N), N > 0 ->
    print_acc_gyro(C),
    % timer:sleep(?LOOP_SLEEP_MS),
    loop_print_acc_gyro(N - 1, C).

% test_pmodnav:print_acc(acc, [out_x_xl, out_y_xl, out_z_xl]).
print_acc(Comp, Registers) ->
    [Ax, Ay, Az] = pmod_nav:read(Comp, Registers),
    io:format(
        "{\"acc\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"acc_ms2\":{\"x\":~p,\"y\":~p,\"z\":~p}}~n",
        [Ax, Ay, Az, Ax * ?G, Ay * ?G, Az * ?G]
    ).

% test_pmodnav:print_gyro(acc, [out_x_g, out_y_g, out_z_g]).
print_gyro(Comp, Registers, C = #cal{}) ->
    [Gx, Gy, Gz] = pmod_nav:read(Comp, Registers),
    print_gyro_values(C, Gx, Gy, Gz).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

print_acc_gyro_values(#cal{gyro = {GBx, GBy, GBz}}, Timestamp, Ax, Ay, Az, Gx, Gy, Gz) ->
    CGx = Gx - GBx,
    CGy = Gy - GBy,
    CGz = Gz - GBz,
    io:format(
        "{\"timestamp\":~p,"
        "\"acc_g\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"acc_ms2\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"gyro\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"gyro_corrected\":{\"x\":~p,\"y\":~p,\"z\":~p}}~n",
        [
            Timestamp,
            Ax, Ay, Az,
            Ax * ?G, Ay * ?G, Az * ?G,
            Gx, Gy, Gz,
            CGx, CGy, CGz
        ]
    ).


print_gyro_values(#cal{gyro = {GBx, GBy, GBz}}, Gx, Gy, Gz) ->
    io:format(
        "{\"gyro\":{\"x\":~p,\"y\":~p,\"z\":~p},"
        "\"gyro_corrected\":{\"x\":~p,\"y\":~p,\"z\":~p}}~n",
        [Gx, Gy, Gz, Gx - GBx, Gy - GBy, Gz - GBz]
    ).




loop_print_acc_angle(0, _C) ->
    ok;
loop_print_acc_angle(N, C) when is_integer(N), N > 0 ->
    [Ax, Ay, Az] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]),
    print_acc_angle(Ax, Ay, Az),
    timer:sleep(?LOOP_SLEEP_MS),
    loop_print_acc_angle(N - 1, C).

round3(X) ->
    round(X * 1000) / 1000.

rad2deg(Rad) ->
    Rad * 180 / math:pi().

print_acc_angle(Ax0, Ay0, Az0) ->
    %% Sensor axes:
    %%   X -> actual Z
    %%   Y -> actual X
    %%   Z -> actual Y
    X = Ay0,
    Y = Az0,
    Z = Ax0,

    Mag = math:sqrt(X * X + Y * Y + Z * Z),

    Roll =
        case Mag of
            0 -> 0;
            _ -> rad2deg(math:atan2(Y, Z))
        end,

    Pitch =
        case Mag of
            0 -> 0;
            _ -> rad2deg(math:atan2(-X, math:sqrt(Y * Y + Z * Z)))
        end,

    Tilt =
        case Mag of
            0 ->
                0;
            _ ->
                Cos = max(-1.0, min(1.0, Z / Mag)),
                rad2deg(math:acos(Cos))
        end,

    io:format(
        "{\"acc\":{\"x\":~.3f,\"y\":~.3f,\"z\":~.3f},"
        "\"angle\":{\"roll\":~.3f,\"pitch\":~.3f,\"tilt\":~.3f}}~n",
        [
            round3(Ax0),
            round3(Ay0),
            round3(Az0),
            round3(Roll),
            round3(Pitch),
            round3(Tilt)
        ]
    ).


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1, N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X) / N, lists:sum(Y) / N, lists:sum(Z) / N].


% C = test_pmodnav:calibrate().
% test_pmodnav:print_acc_gyro(C).
% test_pmodnav:loop_print_acc_gyro(100, C).
% test_pmodnav:loop_print_acc_gyro(10, C).
% test_pmodnav:loop_print_acc_gyro(600, C).
