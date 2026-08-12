-module(ekf4_nav2_uwb).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([
    init/1,
    measure/1
]).

-record(cal, {
    acc,
    gyro,
    t0 = undefined
}).

-record(state, {
    %% Calibration NAV
    cal,
    %% EKF state: [px, py, vx, vy]
    x,
    p,
    %% For the zupt
    stopped_count = 0,
    %% Time of the filter's last iteration
    last_filter_time_ms = undefined,
    %% Prevent reusing same UWB measurement
    last_uwb_seq = undefined
}).


%% Acceleration noise used as model input.
%% Sigma for the Q matrix
-define(SIGMA_ACCEL_INPUT, 0.50).

%% UWB noise in meters.
%% Sigma for the R matrix
-define(SIGMA_UWB, 0.15).

%% For the UWB
-define(UWB_NAME, uwb_measure).
-define(UWB_NODE, 'sensor_fusion@uwb_3').

%% For the ZUPT
-define(ACC_STOP_THRESHOLD, 0.08).
-define(GYRO_STOP_THRESHOLD, 0.250).
-define(STOPPED_MIN_COUNT, 5).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEBUG
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-define(DEBUG, false).
-define(DEBUG_FILE, "uwb_nav_ekf_debug.log").

debug(Fmt, Args) ->
    case ?DEBUG of
        true ->
            Line = io_lib:format(
                "[ekf4_nav2_uwb] [~p] " ++ Fmt ++ "~n",
                [erlang:monotonic_time(millisecond) | Args]
            ),
            file:write_file(?DEBUG_FILE, Line, [append]);
        false ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("ekf4_nav2_uwb (acc): Calibrating... Do not move the pmod_nav!!~n"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("ekf4_nav2_uwb (acc): ~p,~p,~p [g]~n", [Ax,Ay,Az]),
    io:format("ekf4_nav2_uwb (gyro): Calibrating... Do not move the pmod_nav!!~n"),
    [Gx,Gy,Gz] = calibrate(acc, [out_x_g,out_y_g,out_z_g], 300),
    io:format("ekf4_nav2_uwb (gyro): ~p,~p,~p [deg/s]~n", [Gx,Gy,Gz]),
    #cal{acc=[Ax,Ay,Az], gyro=[Gx,Gy,Gz]}.

init(Cal) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 5
    },

    X0 = mat:matrix([
        [0.9], %% px
        [3.15], %% py
        [0.0], %% vx
        [0.0]  %% vy
    ]),

    % X0 = mat:matrix([
    %     [1.7], %% px
    %     [3.15], %% py
    %     [0.0], %% vx
    %     [0.0]  %% vy
    % ]),

    P0 = mat:diag([
        1.0e-6, %% px
        1.0e-6, %% py
        1.0e-4, %% vx
        1.0e-4  %% vy
    ]),

    NowMs = erlang:monotonic_time(millisecond),

    State = #state{
        cal = Cal,
        x = X0,
        p = P0,
        stopped_count = 0,
        last_filter_time_ms = NowMs,
        last_uwb_seq = undefined
    },

    debug("Init", []),

    {ok, State, Spec}.


measure(State0 = #state{
    cal = Cal,
    x = X0,
    p = P0,
    last_filter_time_ms = LastFilterTimeMs
}) ->
    NowMs = erlang:monotonic_time(millisecond),
    Dt = compute_dt_sec(LastFilterTimeMs, NowMs),

    [Ax, Ay, Az, Gx, Gy, Gz] = read_nav2(Cal),

    %% If the NAV Pmod is vertical:
    %% Ay and Az are the horizontal accelerations.
    %% AccX = -Ay,
    %% AccY = Az,

    %% If the NAV Pmod is horizontal:
    %% Ax and Ay are the horizontal accelerations.
    AccX = Ay,
    AccY = Ax,

    {XPred, PPred} = predict_4state(X0, P0, Dt, AccX, AccY),

    StoppedRaw = is_stopped(Ax, Ay, Az, Gx, Gy, Gz),

    {StateAfterStopCount, Stopped} =
        update_stopped_count(State0, StoppedRaw),

    {X1, P1} =
        case Stopped of
            true ->
                apply_zupt_4state(XPred, PPred);
            false ->
                {XPred, PPred}
        end,

    StoppedInt = bool_to_int(Stopped),

    State1 = StateAfterStopCount#state{
        x = X1,
        p = P1,
        last_filter_time_ms = NowMs
    },

    {State2, UwbUpdated, UwbSeq, AnchorId} = try_update_with_uwb(State1),

    UwbUpdatedInt = bool_to_int(UwbUpdated),

    [Spx, Spy, Svx, Svy] = state_to_list(State2#state.x),

    {ok, [Spx, Spy, Svx, Svy, StoppedInt, NowMs, Dt, Ax, Ay, Az, Gx, Gy, Gz, UwbUpdatedInt, UwbSeq, AnchorId], State2}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EKF 4 states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predict_4state(X0, P0, Dt, AccX, AccY) ->
    Sigma2 = ?SIGMA_ACCEL_INPUT * ?SIGMA_ACCEL_INPUT,
    Dt2 = Dt * Dt,

    %% Process noise caused by uncertainty in acceleration.
    Q = mat:matrix([
        [0.25 * Dt2 * Dt2 * Sigma2, 0.0, 0.5 * Dt * Dt2 * Sigma2, 0.0],
        [0.0, 0.25 * Dt2 * Dt2 * Sigma2, 0.0, 0.5 * Dt * Dt2 * Sigma2],
        [0.5 * Dt * Dt2 * Sigma2, 0.0, Dt2 * Sigma2, 0.0],
        [0.0, 0.5 * Dt * Dt2 * Sigma2, 0.0, Dt2 * Sigma2]
    ]),

    %% Nonlinear/affine prediction function:
    %% x' = f(x, u)
    %% Here, u = [AccX, AccY]
    FFun =
        fun(X) ->
            Px0 = mat:get(1, 1, X),
            Py0 = mat:get(2, 1, X),
            Vx0 = mat:get(3, 1, X),
            Vy0 = mat:get(4, 1, X),

            Px1 = Px0 + Vx0 * Dt + 0.5 * AccX * Dt2,
            Py1 = Py0 + Vy0 * Dt + 0.5 * AccY * Dt2,
            Vx1 = Vx0 + AccX * Dt,
            Vy1 = Vy0 + AccY * Dt,

            mat:matrix([
                [Px1],
                [Py1],
                [Vx1],
                [Vy1]
            ])
        end,

    %% Jacobian of f with respect to the state x.
    %% Acceleration is an external input, so it does not appear
    %% in the Jacobian with respect to x.
    JFFun =
        fun(_X) ->
            mat:matrix([
                [1.0, 0.0, Dt,  0.0],
                [0.0, 1.0, 0.0, Dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
        end,

    hera2:ekf_predict({X0, P0}, FFun, JFFun, Q).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ZUPT 4 states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

apply_zupt_4state(X, P) ->
    [Px, Py, _Vx, _Vy] = state_to_list(X),

    X1 = mat:matrix([
        [Px],
        [Py],
        [0.0],
        [0.0]
    ]),

    %% We retain the position uncertainty.
    %% We reduce the uncertainty in vx/vy.
    %% We remove the position-velocity correlations to prevent a position jump.
    P1 = mat:matrix([
        [mat:get(1, 1, P), 0.0, 0.0, 0.0],
        [0.0, mat:get(2, 2, P), 0.0, 0.0],
        [0.0, 0.0, 1.0e-5, 0.0],
        [0.0, 0.0, 0.0, 1.0e-5]
    ]),

    {X1, P1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UWB update, one anchor measurement at a time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try_update_with_uwb(State = #state{last_uwb_seq = LastSeq}) ->
    case hera_data:get(?UWB_NAME, ?UWB_NODE) of
        [{_Node, Seq, _Timestamp, Data}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    update_from_uwb_sample(Seq, Data, State);
                false ->
                    {State, false, LastSeq, -1}
            end;
        _ ->
            {State, false, LastSeq, -1}
    end.


update_from_uwb_sample(Seq, Data, State = #state{x = X0, p = P0}) ->
    %% Expected UWB data:
    %% [AnchorId, DistanceCm, AnchorX, AnchorY]
    [AnchorId, DistanceCm, AnchorX, AnchorY] = Data,
    DistanceM = DistanceCm / 100.0,

    {HFun, JHFun, R, Z} =
        uwb_range_measurement_model(DistanceM, AnchorX, AnchorY, ?SIGMA_UWB),

    {X1, P1} = hera2:ekf_update({X0, P0}, HFun, JHFun, R, Z),

    % debug("uwb update seq=~p anchor=~p dist_m=~p", [Seq, AnchorId, DistanceM]),

    {State#state{x = X1, p = P1, last_uwb_seq = Seq}, true, Seq, AnchorId}.


uwb_range_measurement_model(DistanceM, AnchorX, AnchorY, SigmaUwb) ->
    %% h(X) = sqrt((px-anchor_x)^2 + (py-anchor_y)^2)
    HFun =
        fun(X) ->
            {RPred, _Dx, _Dy} = predicted_range(X, AnchorX, AnchorY),
            mat:matrix([[RPred]])
        end,

    %% Jacobian:
    %% H = [(px-anchor_x)/r, (py-anchor_y)/r, 0, 0]
    JHFun =
        fun(X) ->
            {RPred, Dx, Dy} = predicted_range(X, AnchorX, AnchorY),
            mat:matrix([[Dx / RPred, Dy / RPred, 0.0, 0.0]])
        end,

    R = mat:matrix([[SigmaUwb * SigmaUwb]]),
    Z = mat:matrix([[DistanceM]]),

    {HFun, JHFun, R, Z}.


predicted_range(X, AnchorX, AnchorY) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),
    Dx = Px - AnchorX,
    Dy = Py - AnchorY,
    R0 = math:sqrt(Dx * Dx + Dy * Dy),
    %% Prevents division by zero.
    R = max(R0, 0.000001),
    {R, Dx, Dy}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_to_list(X) ->
    [
        mat:get(1, 1, X),
        mat:get(2, 1, X),
        mat:get(3, 1, X),
        mat:get(4, 1, X)
    ].

is_new_seq(_Seq, undefined) ->
    true;
is_new_seq(Seq, LastSeq) ->
    Seq =/= LastSeq.

clamp_dt(Dt) when Dt =< 0.0 ->
    0.004;
clamp_dt(Dt) when Dt > 0.1 ->
    0.1;
clamp_dt(Dt) ->
    Dt.

is_stopped(Ax, Ay, Az, Gx, Gy, Gz) ->
    abs_float(Ax) =< ?ACC_STOP_THRESHOLD andalso
    abs_float(Ay) =< ?ACC_STOP_THRESHOLD andalso
    abs_float(Az) =< ?ACC_STOP_THRESHOLD andalso
    abs_float(Gx) =< ?GYRO_STOP_THRESHOLD andalso
    abs_float(Gy) =< ?GYRO_STOP_THRESHOLD andalso
    abs_float(Gz) =< ?GYRO_STOP_THRESHOLD.

update_stopped_count(State = #state{stopped_count = Count}, StoppedRaw) ->
    Count1 =
        case StoppedRaw of
            true ->
                Count + 1;
            false ->
                0
        end,

    Stopped = Count1 >= ?STOPPED_MIN_COUNT,

    {State#state{stopped_count = Count1}, Stopped}.

% stopped_int(#state{stopped_count = Count}) ->
%     case Count >= ?STOPPED_MIN_COUNT of
%         true -> 1;
%         false -> 0
%     end.

abs_float(X) when X < 0 ->
    -X;
abs_float(X) ->
    X.

bool_to_int(true) ->
    1;
bool_to_int(false) ->
    0.

compute_dt_sec(undefined, _NowMs) ->
    0.01;
compute_dt_sec(LastMs, NowMs) ->
    Dt = (NowMs - LastMs) / 1000.0,
    clamp_dt(Dt).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers nav
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% read_nav(C) ->
%     [Ax,Ay,Az, Gx,Gy,Gz] = pmod_nav:read(acc, [
%         out_x_xl,out_y_xl,out_z_xl,
%         out_x_g,out_y_g,out_z_g]),
%     Acc = subtract([Ax, Ay, Az], C#cal.acc),
%     Gyro = subtract([Gx, Gy, Gz], C#cal.gyro),
%     %% acc #{xl_unit => g} => m/s^2
%     [Axx, Ayy, Azz] = scale(Acc, 9.81),
%     %% gyro #{g_unit => dps} => dps
%     [Gxx, Gyy, Gzz] = Gyro,
%     [Axx, Ayy, Azz, Gxx, Gyy, Gzz].

read_nav2(C) ->
    [Ax,Ay, Gx,Gy] = pmod_nav:read(acc, [
        out_x_xl,out_y_xl,
        out_x_g,out_y_g]),
    Az = 0,
    Gz = 0,
    Acc = subtract([Ax, Ay, Az], C#cal.acc),
    Gyro = subtract([Gx, Gy, Gz], C#cal.gyro),
    %% acc #{xl_unit => g} => m/s^2
    [Axx, Ayy, _] = scale(Acc, 9.81),
    %% gyro #{g_unit => dps} => dps
    [Gxx, Gyy, _] = Gyro,
    [Axx, Ayy, 0, Gxx, Gyy, 0].

scale(List, Factor) ->
    [X*Factor || X <- List].

calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].

subtract([X,Y,Z], [X0,Y0,Z0]) ->
    [X-X0, Y-Y0, Z-Z0].
