-module(ekf2_uwb).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    %% EKF state: [px, py]
    x,
    p,
    last_uwb_seq = undefined
}).

-define(TIMEOUT_MS, 10).
-define(SIGMA_UWB, 0.10).

-define(UWB_NAME, uwb_measure).
-define(UWB_NODE, 'sensor_fusion@uwb_3').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(_) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => ?TIMEOUT_MS
    },

    %% Important : ne pas initialiser exactement sur une ancre.
    %% Mets par exemple le centre approximatif de ta zone.
    X0 = mat:matrix([
        [1.0], %% px
        [1.0]  %% py
    ]),

    P0 = mat:diag([
        1.0,
        1.0
    ]),

    State = #state{
        x = X0,
        p = P0,
        last_uwb_seq = undefined
    },

    {ok, State, Spec}.

measure(State0 = #state{x = X0, last_uwb_seq = LastSeq}) ->
    case hera_data:get(?UWB_NAME, ?UWB_NODE) of
        [{_Node, Seq, _Timestamp, Data}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    {State1, UwbUpdated, AnchorId} =
                        update_from_uwb_sample(Seq, Data, State0),

                    [Px, Py] = state_to_list(State1#state.x),

                    UwbUpdatedInt = bool_to_int(UwbUpdated),

                    {ok, [Px, Py, UwbUpdatedInt, Seq, AnchorId], State1};

                false ->
                    [Px, Py] = state_to_list(X0),
                    {ok, [Px, Py, 0, LastSeq, -1], State0}
            end;

        _ ->
            [Px, Py] = state_to_list(X0),
            {ok, [Px, Py, 0, -1, -1], State0}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UWB update, one anchor measurement at a time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

update_from_uwb_sample(Seq, Data, State = #state{x = X0, p = P0}) ->
    %% Expected UWB data:
    %% [AnchorId, DistanceCm, AnchorX, AnchorY]
    [AnchorId, DistanceCm, AnchorX, AnchorY] = Data,

    DistanceM = DistanceCm / 100.0,

    %% Si AnchorX/AnchorY sont déjà en mètres, garde ceci.
    AnchorXM = AnchorX,
    AnchorYM = AnchorY,

    {HFun, JHFun, R, Z} =
        uwb_range_measurement_model_2state(
            DistanceM, AnchorXM, AnchorYM, ?SIGMA_UWB),

    % P0Noise = add_process_noise_2state(P0),
    % {X1, P1} = hera2:ekf_update({X0, P0Noise}, HFun, JHFun, R, Z),
    {X1, P1} = hera2:ekf_update({X0, P0}, HFun, JHFun, R, Z),

    {State#state{x = X1, p = P1, last_uwb_seq = Seq}, true, AnchorId}.

uwb_range_measurement_model_2state(DistanceM, AnchorX, AnchorY, SigmaUwb) ->
    %% h(X) = sqrt((px-anchor_x)^2 + (py-anchor_y)^2)
    HFun =
        fun(X) ->
            {RPred, _Dx, _Dy} = predicted_range_2state(X, AnchorX, AnchorY),
            mat:matrix([[RPred]])
        end,

    %% Jacobian:
    %% H = [(px-anchor_x)/r, (py-anchor_y)/r]
    JHFun =
        fun(X) ->
            {RPred, Dx, Dy} = predicted_range_2state(X, AnchorX, AnchorY),
            mat:matrix([
                [Dx / RPred, Dy / RPred]
            ])
        end,

    R = mat:matrix([[SigmaUwb * SigmaUwb]]),
    Z = mat:matrix([[DistanceM]]),

    {HFun, JHFun, R, Z}.

predicted_range_2state(X, AnchorX, AnchorY) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),
    Dx = Px - AnchorX,
    Dy = Py - AnchorY,
    R0 = math:sqrt(Dx * Dx + Dy * Dy),

    %% Évite division par zéro si on est exactement sur l'ancre.
    R = max_float(R0, 0.000001),

    {R, Dx, Dy}.

add_process_noise_2state(P0) ->
    Q = mat:diag([
        1.0e-4,
        1.0e-4
    ]),
    mat:'+'(P0, Q).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_to_list(X) ->
    [
        mat:get(1, 1, X),
        mat:get(2, 1, X)
    ].


is_new_seq(_Seq, undefined) ->
    true;
is_new_seq(Seq, LastSeq) ->
    Seq =/= LastSeq.


bool_to_int(true) ->
    1;
bool_to_int(false) ->
    0.


max_float(A, B) when A >= B ->
    A;
max_float(_A, B) ->
    B.