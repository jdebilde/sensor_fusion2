-module(heading_mag).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    mag_bias = [0.0, 0.0, 0.0],
    yaw0 = undefined,
    t0 = undefined
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    %% Assumption: The PmodNAV X axis is vertical.
    %% x -> Z: Yaw
    %% y -> X: Pitch
    %% z -> Y: Roll
    io:format("nav1_mag (mag): Place de pmod_nav flat and still!~n"),
    [Mx1,My1,Mz1] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 30),
    _ = io:get_line("nav1_mag (mag): Turn the pmod_nav 180° around the z axis then press enter"),
    [_,My2,Mz2] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 30),
    _ = io:get_line("nav1_mag (mag): Turn the pmod_nav 180° around the x axis then press enter"),
    [Mx2,_,_] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 30),
    BiasX = 0.5*(Mx1+Mx2),
    BiasY = 0.5*(My1+My2),
    BiasZ = 0.5*(Mz1+Mz2),
    io:format("nav1_mag (mag): BiasX = ~p~n", [BiasX]),
    io:format("nav1_mag (mag): BiasY = ~p~n", [BiasY]),
    io:format("nav1_mag (mag): BiasZ = ~p~n", [BiasZ]),
    #cal{
        mag_bias=[BiasX,BiasY,BiasZ]
    }.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(C) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 10
    },
    T0 = erlang:monotonic_time(microsecond),
    %% default unit: gauss.
    [Mx, My, Mz] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    [_, Myy, Mzz] = subtract([Mx, My, Mz], C#cal.mag_bias),

    Yaw0 = yaw_from_yz(Myy, Mzz),

    {ok, C#cal{yaw0 = Yaw0, t0 = T0}, Spec}.

measure(C = #cal{yaw0 = Yaw0, t0 = T0}) ->
    T1 = erlang:monotonic_time(microsecond),
    Dt =
        case T0 of
            undefined -> 0.0;
            _ -> (T1 - T0) / 1000000.0
        end,

    [Mx0, My0, Mz0] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    [Mx, My, Mz] = subtract([Mx0, My0, Mz0], C#cal.mag_bias),

    %% Because of your physical mounting:
    %%   yaw axis = IMU X
    %%   yaw plane = IMU Y/Z
    YawAbs = yaw_from_yz(My, Mz),
    YawRel = wrap_180(YawAbs - Yaw0),

    %% Diagnostic angles. If YawRel behaves badly, compare these while rotating.
    YawXY = yaw_from_xy(Mx, My),
    YawZX = yaw_from_zx(Mz, Mx),

    Data = [T1, Dt, YawRel, YawAbs, YawXY, YawZX, Mx, My, Mz],
    {ok, Data, C#cal{t0 = T1}}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate(Comp, Registers, N) ->
    Data = [
        list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1, N)
    ],
    {X, Y, Z} = lists:unzip3(Data),
    [
        lists:sum(X) / N,
        lists:sum(Y) / N,
        lists:sum(Z) / N
    ].

subtract([X, Y, Z], [X0, Y0, Z0]) ->
    [X - X0, Y - Y0, Z - Z0].

%% Yaw when IMU X is approximately vertical.
yaw_from_yz(My, Mz) ->
    wrap_360(rad_to_deg(math:atan2(Mz, My))).

%% Diagnostic: yaw when IMU Z is approximately vertical.
yaw_from_xy(Mx, My) ->
    wrap_360(rad_to_deg(math:atan2(My, Mx))).

%% Diagnostic: yaw when IMU Y is approximately vertical.
yaw_from_zx(Mz, Mx) ->
    wrap_360(rad_to_deg(math:atan2(Mx, Mz))).

rad_to_deg(Rad) ->
    Rad * 180.0 / math:pi().

wrap_360(A) when A >= 360.0 ->
    wrap_360(A - 360.0);
wrap_360(A) when A < 0.0 ->
    wrap_360(A + 360.0);
wrap_360(A) ->
    A.

wrap_180(A) when A > 180.0 ->
    wrap_180(A - 360.0);
wrap_180(A) when A =< -180.0 ->
    wrap_180(A + 360.0);
wrap_180(A) ->
    A.
