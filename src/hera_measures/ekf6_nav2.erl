-module(ekf6_nav2).

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
    last_filter_time_ms = undefined
}).

%% Sigma for the Q matrix
-define(SIGMA_ACCEL_STATE, 0.50).
%% Sigma for the R matrix
-define(SIGMA_ACCEL_MEAS, 0.20).

%% For the zupt
-define(ACC_STOP_THRESHOLD, 0.08).
-define(GYRO_STOP_THRESHOLD, 0.250).
-define(STOPPED_MIN_COUNT, 5).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEBUG
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -define(DEBUG, true).
% -define(DEBUG_FILE, "debug.log").

% debug(Fmt, Args) ->
%     case ?DEBUG of
%         true ->
%             Line = io_lib:format(
%                 "[ekf6_nav2] [~p] " ++ Fmt ++ "~n",
%                 [erlang:monotonic_time(millisecond) | Args]
%             ),
%             file:write_file(?DEBUG_FILE, Line, [append]);
%         false ->
%             ok
%     end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("ekf6_nav2 (acc): Calibrating... Do not move the pmod_nav!!~n"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("ekf6_nav2 (acc): ~p,~p,~p [g]~n", [Ax,Ay,Az]),
    io:format("ekf6_nav2 (gyro): Calibrating... Do not move the pmod_nav!!~n"),
    [Gx,Gy,Gz] = calibrate(acc, [out_x_g,out_y_g,out_z_g], 300),
    io:format("ekf6_nav2 (gyro): ~p,~p,~p [deg/s]~n", [Gx,Gy,Gz]),
    #cal{acc=[Ax,Ay,Az], gyro=[Gx,Gy,Gz]}.

init(Cal) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 0
    },

    X0 = mat:matrix([
        [0.0], %% px
        [0.0], %% py
        [0.0], %% vx
        [0.0], %% vy
        [0.0], %% ax
        [0.0]  %% ay
    ]),

    P0 = mat:diag([
        1.0e-6, %% px
        1.0e-6, %% py
        1.0e-4, %% vx
        1.0e-4, %% vy
        1.0,    %% ax
        1.0     %% ay
    ]),

    NowMs = erlang:monotonic_time(millisecond),

    State = #state{
        cal = Cal,
        x = X0,
        p = P0,
        stopped_count = 0,
        last_filter_time_ms = NowMs
    },

    % debug("Init", []),

    {ok, State, Spec}.


measure(State0 = #state{
    cal = Cal,
    x = X0,
    p = P0,
    last_filter_time_ms = LastFilterTimeMs
}) ->
    NowMs = erlang:monotonic_time(millisecond),
    Dt = compute_dt_sec(LastFilterTimeMs, NowMs),

    {XPred, PPred} = predict_6state(X0, P0, Dt),

    StatePred = State0#state{
        x = XPred,
        p = PPred,
        last_filter_time_ms = NowMs
    },

    [Ax, Ay, Az, Gx, Gy, Gz] = read_nav2(Cal),

    {State1, StoppedInt} = update_with_nav([Ax, Ay, Az, Gx, Gy, Gz], StatePred),

    [Spx, Spy, Svx, Svy, Sax, Say] = state_to_list(State1#state.x),

    {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt, NowMs, Dt, Ax, Ay, Az, Gx, Gy, Gz], State1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EKF 6 states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predict_6state(X0, P0, Dt) ->
    Dt2 = Dt * Dt,

    Q = mat:diag([
        1.0e-8,
        1.0e-8,
        1.0e-6,
        1.0e-6,
        ?SIGMA_ACCEL_STATE * ?SIGMA_ACCEL_STATE,
        ?SIGMA_ACCEL_STATE * ?SIGMA_ACCEL_STATE
    ]),

    %% Fonction de prédiction :
    %% état = [px, py, vx, vy, ax, ay]
    FFun =
        fun(X) ->
            Px0 = mat:get(1, 1, X),
            Py0 = mat:get(2, 1, X),
            Vx0 = mat:get(3, 1, X),
            Vy0 = mat:get(4, 1, X),
            Ax0 = mat:get(5, 1, X),
            Ay0 = mat:get(6, 1, X),

            Px1 = Px0 + Vx0 * Dt + 0.5 * Ax0 * Dt2,
            Py1 = Py0 + Vy0 * Dt + 0.5 * Ay0 * Dt2,
            Vx1 = Vx0 + Ax0 * Dt,
            Vy1 = Vy0 + Ay0 * Dt,

            mat:matrix([
                [Px1],
                [Py1],
                [Vx1],
                [Vy1],
                [Ax0],
                [Ay0]
            ])
        end,

    %% Jacobien de f(x)
    JFFun =
        fun(_X) ->
            mat:matrix([
                [1.0, 0.0, Dt,  0.0, 0.5 * Dt2, 0.0],
                [0.0, 1.0, 0.0, Dt,  0.0, 0.5 * Dt2],
                [0.0, 0.0, 1.0, 0.0, Dt,  0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, Dt],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            ])
        end,

    hera2:ekf_predict({X0, P0}, FFun, JFFun, Q).

update_with_nav([Ax, Ay, Az, Gx, Gy, Gz], State0 = #state{x = XPred, p = PPred}) ->

    R = mat:diag([
        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS,
        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS
    ]),

    %% If the NAV Pmod is vertical:
    %% Ay and Az are the horizontal accelerations.
    %% AccX = -Ay,
    %% AccY = Az,
    % Z = mat:matrix([
    %     [-Ay],
    %     [Az]
    % ]),

    %% If the NAV Pmod is horizontal:
    %% Ax and Ay are the horizontal accelerations.
    %% AccX = Ay,
    %% AccY = Ax,
    Z = mat:matrix([
        [Ay],
        [Ax]
    ]),

    %% h_nav(x) = [ax, ay]
    HFun =
        fun(X) ->
            mat:matrix([
                [mat:get(5, 1, X)],
                [mat:get(6, 1, X)]
            ])
        end,

    %% Jacobien de h_nav(x)
    JHFun =
        fun(_X) ->
            mat:matrix([
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            ])
        end,

    {XUpdate, PUpdate} = hera2:ekf_update({XPred, PPred}, HFun, JHFun, R, Z),

    %% With abrupt stop detection.
    StoppedRaw = is_stopped(Ax, Ay, Az, Gx, Gy, Gz),
    %% Without abrupt stop detection.
    % StoppedRaw = false,

    {StateAfterStopCount, Stopped} =
        update_stopped_count(State0, StoppedRaw),

    {X1, P1} =
        case Stopped of
            true ->
                apply_zupt_6state(XUpdate, PUpdate);
            false ->
                {XUpdate, PUpdate}
        end,

    StoppedInt = bool_to_int(Stopped),

    State1 = StateAfterStopCount#state{
        x = X1,
        p = P1
    },

    {State1, StoppedInt}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ZUPT 6 states
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

apply_zupt_6state(X, P) ->
    [Px, Py, _Vx, _Vy, _Ax, _Ay] = state_to_list(X),

    X1 = mat:matrix([
        [Px],
        [Py],
        [0.0],
        [0.0],
        [0.0],
        [0.0]
    ]),

    {X1, P}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_to_list(X) ->
    [
        mat:get(1, 1, X),
        mat:get(2, 1, X),
        mat:get(3, 1, X),
        mat:get(4, 1, X),
        mat:get(5, 1, X),
        mat:get(6, 1, X)
    ].

% is_new_seq(_Seq, undefined) ->
%     true;
% is_new_seq(Seq, LastSeq) ->
%     Seq =/= LastSeq.

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
