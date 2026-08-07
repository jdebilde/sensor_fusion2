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
    stopped_count = 0,
    %% Pour prédire à chaque itération du filtre
    last_filter_time_ms = undefined
}).

-define(SIGMA_ACCEL_MEAS, 0.20).
-define(SIGMA_ACCEL_STATE, 0.50).

%% For the zupt
-define(ACC_STOP_THRESHOLD, 0.08).
-define(GYRO_STOP_THRESHOLD, 0.250).
-define(STOPPED_MIN_COUNT, 5).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEBUG
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% -define(DEBUG, true).
% -define(DEBUG_FILE, "uwb_nav_ekf_debug.log").

% debug(Fmt, Args) ->
%     case ?DEBUG of
%         true ->
%             Line = io_lib:format(
%                 "[nav2_ekf] [~p] " ++ Fmt ++ "~n",
%                 [erlang:monotonic_time(millisecond) | Args]
%             ),
%             file:write_file(?DEBUG_FILE, Line, [append]);
%         false ->
%             ok
%     end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(_) ->
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
        x = X0,
        p = P0,
        last_nav_seq = undefined,
        stopped_count = 0,
        last_filter_time_ms = NowMs
    },
    % debug("Init", []),

    {ok, State, Spec}.


measure(State0 = #state{
    x = X0,
    p = P0,
    last_nav_seq = LastSeq,
    last_filter_time_ms = LastFilterTimeMs
}) ->
    %% 1. Prédiction à chaque itération du filtre
    NowMs = erlang:monotonic_time(millisecond),
    DtPred = filter_dt(LastFilterTimeMs, NowMs),

    {XPred, PPred} = predict_6state(X0, P0, DtPred),

    StatePred = State0#state{
        x = XPred,
        p = PPred,
        last_filter_time_ms = NowMs
    },

    %% 2. Si une nouvelle mesure NAV existe, update.
    %%    Sinon, on retourne seulement la prédiction.
    case hera_data:get(nav2, node()) of
        [{_Node, Seq, _Heratimestamp, NavData}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    {State1, StoppedInt} = update_with_nav(Seq, NavData, StatePred),
                    [Spx, Spy, Svx, Svy, Sax, Say] = state_to_list(State1#state.x),

                    {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt, Seq], State1};

                false ->
                    %% Pas de nouvelle mesure NAV.
                    %% On garde quand même la prédiction.
                    [Spx, Spy, Svx, Svy, Sax, Say] = state_to_list(XPred),
                    StoppedInt = stopped_int(StatePred),

                    {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt, LastSeq], StatePred}
            end;

        _ ->
            %% Aucune mesure NAV disponible.
            %% On retourne aussi la prédiction.
            [Spx, Spy, Svx, Svy, Sax, Say] = state_to_list(XPred),
            StoppedInt = stopped_int(StatePred),

            {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt, -1], StatePred}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
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

update_with_nav(Seq, [Navtimestamp, _DtNav, Ax, Ay, Az, Gx, Gy, Gz],
                State0 = #state{x = XPred, p = PPred}) ->
    _ = Navtimestamp,

    R = mat:diag([
        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS,
        ?SIGMA_ACCEL_MEAS * ?SIGMA_ACCEL_MEAS
    ]),

    %% Si le pmodNav est vertical
    %% AccX = -Ay,
    %% AccY = Az,
    % Z = mat:matrix([
    %     [-Ay],
    %     [Az]
    % ]),

    %% Si le pmodNAV est horizontal :
    %% AccX = Ay
    %% AccY = Ax
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

    %% Avec détection arrêt brut.
    StoppedRaw = is_stopped(Ax, Ay, Az, Gx, Gy, Gz),
    %% Sans détection arrêt brut.
    % StoppedRaw = false,

    {StateAfterStopCount, Stopped} =
        update_stopped_count(State0, StoppedRaw),

    X1 =
        case Stopped of
            true ->
                apply_zupt_6state(XUpdate);
            false ->
                XUpdate
        end,

    StoppedInt = bool_to_int(Stopped),

    State1 = StateAfterStopCount#state{
        x = X1,
        p = PUpdate,
        last_nav_seq = Seq
    },

    {State1, StoppedInt}.

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

stopped_int(#state{stopped_count = Count}) ->
    case Count >= ?STOPPED_MIN_COUNT of
        true -> 1;
        false -> 0
    end.

abs_float(X) when X < 0 ->
    -X;
abs_float(X) ->
    X.

bool_to_int(true) ->
    1;
bool_to_int(false) ->
    0.

filter_dt(undefined, _NowMs) ->
    0.01;
filter_dt(LastMs, NowMs) ->
    Dt = (NowMs - LastMs) / 1000.0,
    clamp_dt(Dt).