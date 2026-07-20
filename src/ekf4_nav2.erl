-module(ekf4_nav2).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    %% EKF state: [px, py, vx, vy]
    x,
    p,
    %% Prevent reusing same Hera measurements
    last_nav_seq = undefined,
    %% For the zupt
    stopped_count = 0
}).

-define(TIMEOUT_MS, 10).

%% Bruit d'accélération utilisée comme entrée du modèle.
-define(SIGMA_ACCEL_INPUT, 0.50).

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
                "[nav2_ekf2] [~p] " ++ Fmt ++ "~n",
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
        [0.0]  %% vy
    ]),

    P0 = mat:diag([
        1.0e-6, %% px
        1.0e-6, %% py
        1.0e-4, %% vx
        1.0e-4  %% vy
    ]),

    State = #state{
        x = X0,
        p = P0,
        last_nav_seq = undefined,
        stopped_count = 0
    },

    debug("Init", []),

    {ok, State, Spec}.


measure(State0 = #state{x = X0, p = P0, last_nav_seq = LastSeq}) ->
    case hera_data:get(nav2, node()) of
        [{_Node, Seq, _Heratimestamp, [_Navtimestamp, Dt0, Ax, Ay, Az, Gx, Gy, Gz]}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    Dt = clamp_dt(Dt0),

                    %% Dans ton cas, Ay et Az sont les accélérations horizontales.
                    AccX = -Ay,
                    AccY = Az,

                    %% Détection arrêt brut.
                    StoppedRaw = is_stopped(Ax, Ay, Az, Gx, Gy, Gz),

                    {StateAfterStopCount, Stopped} =
                        update_stopped_count(State0, StoppedRaw),

                    %% Prédiction 4 états avec accélération comme entrée.
                    {XPred, PPred} = predict_4state(X0, P0, Dt, AccX, AccY),

                    %% ZUPT après 5 mesures consécutives immobiles.
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
                        last_nav_seq = Seq
                    },

                    [Spx, Spy, Svx, Svy] = state_to_list(X1),

                    {ok, [Spx, Spy, Svx, Svy, StoppedInt, Seq], State1};

                false ->
                    [Spx, Spy, Svx, Svy] = state_to_list(X0),
                    StoppedInt = stopped_int(State0),
                    {ok, [Spx, Spy, Svx, Svy, StoppedInt, LastSeq], State0}
            end;

        _ ->
            [Spx, Spy, Svx, Svy] = state_to_list(X0),
            StoppedInt = stopped_int(State0),
            {ok, [Spx, Spy, Svx, Svy, StoppedInt, -1], State0}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EKF 4 états
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

predict_4state(X0, P0, Dt, AccX, AccY) ->
    Sigma2 = ?SIGMA_ACCEL_INPUT * ?SIGMA_ACCEL_INPUT,
    Dt2 = Dt * Dt,

    %% Bruit de processus causé par l'incertitude sur l'accélération.
    Q = mat:matrix([
        [0.25 * Dt2 * Dt2 * Sigma2, 0.0, 0.5 * Dt * Dt2 * Sigma2, 0.0],
        [0.0, 0.25 * Dt2 * Dt2 * Sigma2, 0.0, 0.5 * Dt * Dt2 * Sigma2],
        [0.5 * Dt * Dt2 * Sigma2, 0.0, Dt2 * Sigma2, 0.0],
        [0.0, 0.5 * Dt * Dt2 * Sigma2, 0.0, Dt2 * Sigma2]
    ]),

    %% Fonction de prédiction non-linéaire/affine :
    %% x' = f(x, u)
    %% Ici u = [AccX, AccY] est capturé par la closure Erlang.
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

    %% Jacobien de f par rapport à l'état x.
    %% L'accélération est une entrée externe, donc elle n'apparaît pas
    %% dans le Jacobien par rapport à x.
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
%% ZUPT 4 états
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

apply_zupt_4state(X, P) ->
    [Px, Py, _Vx, _Vy] = state_to_list(X),

    X1 = mat:matrix([
        [Px],
        [Py],
        [0.0],
        [0.0]
    ]),

    %% On garde l'incertitude de position.
    %% On réduit l'incertitude sur vx/vy.
    %% On coupe les corrélations position-vitesse pour éviter un saut de position.
    P1 = mat:matrix([
        [mat:get(1, 1, P), 0.0, 0.0, 0.0],
        [0.0, mat:get(2, 2, P), 0.0, 0.0],
        [0.0, 0.0, 1.0e-5, 0.0],
        [0.0, 0.0, 0.0, 1.0e-5]
    ]),

    {X1, P1}.

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