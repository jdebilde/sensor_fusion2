-module(ekf6_nav2).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    %% EKF state
    x,
    p,
    %% Prevent reusing same Hera measurements
    last_nav_seq = undefined,
    %% For the zupt
    stopped_count = 0
}).

-define(TIMEOUT_MS, 10).
-define(SIGMA_ACCEL_MEAS, 0.20).
-define(SIGMA_ACCEL_STATE, 0.50).

%% For the zupt
-define(ACC_STOP_THRESHOLD, 0.08).
-define(GYRO_STOP_THRESHOLD, 0.150).
-define(STOPPED_MIN_COUNT, 5).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEBUG
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-define(DEBUG, true).
-define(DEBUG_FILE, "uwb_nav_ekf_debug.log").

debug(Fmt, Args) ->
    case ?DEBUG of
        true ->
            Line = io_lib:format(
                "[nav2_ekf] [~p] " ++ Fmt ++ "~n",
                [erlang:monotonic_time(millisecond) | Args]
            ),
            file:write_file(?DEBUG_FILE, Line, [append]);
        false ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(_) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => ?TIMEOUT_MS
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

    State = #state{
        x = X0,
        p = P0,
        last_nav_seq = undefined
    },
    debug("Init", []),

    {ok, State, Spec}.


measure(State0 = #state{x = X0, p = P0, last_nav_seq = LastSeq}) ->
    case hera_data:get(nav2, node()) of
        [{_Node, Seq, _Heratimestamp, [Navtimestamp, Dt, Ax,Ay,Az, Gx,Gy,Gz]}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    % debug("new mesure", []),
                    F = mat:matrix([
                        [1.0, 0.0, Dt,  0.0, 0.5 * Dt * Dt, 0.0],
                        [0.0, 1.0, 0.0, Dt,  0.0, 0.5 * Dt * Dt],
                        [0.0, 0.0, 1.0, 0.0, Dt,  0.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, Dt],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                    ]),

                    Q = mat:diag([
                        1.0e-8,
                        1.0e-8,
                        1.0e-6,
                        1.0e-6,
                        ?SIGMA_ACCEL_STATE * ?SIGMA_ACCEL_STATE,
                        ?SIGMA_ACCEL_STATE * ?SIGMA_ACCEL_STATE
                    ]),

                    % debug("kf_predict", []),
                    {XPred, PPred} = kalman:kf_predict({X0, P0}, F, Q),

                    %% Mesure : z = [acc_x, acc_y]^T observe les états ax, ay.
                    H = mat:matrix([
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                    ]),

                    R = mat:diag([
                        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS,
                        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS
                    ]),

                    Z = mat:matrix([
                        [Ay],
                        [Az]
                    ]),

                    % debug("kf_update", []),
                    {XUpdate, P1} = kalman:kf_update({XPred, PPred}, H, R, Z),

                    Stopped0 = is_stopped(Ax, Ay, Az, Gx, Gy, Gz),

                    {StateAfterStopCount, Stopped} =
                        update_stopped_count(State0, Stopped0),

                    X1 =
                        case Stopped of
                            true ->
                                apply_zupt_6state(XUpdate);
                            false ->
                                XUpdate
                        end,

                    StoppedInt = bool_to_int(Stopped),

                    % debug("return", []),
                    State1 = StateAfterStopCount#state{
                        x = X1,
                        p = P1,
                        last_nav_seq = Seq
                    },

                    Values = state_to_list(X1),
                    [Spx, Spy, Svx, Svy, Sax, Say] = Values,

                    % debug("{ok, ~p, ~p}", [Values, State1]),
                    {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt, Seq], State1};

                false ->
                    debug("not new mesure", []),
                    {undefined, State0}
            end;
        _ ->
            debug("no mesure", []),
            {undefined, State0}
    end.

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


is_new_seq(_Seq, undefined) ->
    true;
is_new_seq(Seq, LastSeq) ->
    Seq =/= LastSeq.


clamp_dt(Dt) when Dt =< 0.0 ->
    0.004;
clamp_dt(Dt) when Dt > 0.1 ->
    %% Avoid huge jumps after pauses/debugging.
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

apply_zupt_6state(X) ->
    [Px, Py, _Vx, _Vy, _Ax, _Ay] = state_to_list(X),

    mat:matrix([
        [Px],
        [Py],
        [0.0],
        [0.0],
        [0.0],
        [0.0]
    ]).

abs_float(X) when X < 0 ->
    -X;
abs_float(X) ->
    X.

bool_to_int(true) ->
    1;
bool_to_int(false) ->
    0.