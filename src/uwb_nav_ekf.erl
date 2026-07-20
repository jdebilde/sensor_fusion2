-module(uwb_nav_ekf).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    %% EKF state
    x,                  %% mat 4x1: [px, py, vx, vy]^T
    p,                  %% mat 4x4 covariance

    %% Timing
    last_nav_t_us = undefined,

    %% Hera sources
    nav_name = nav3,
    nav_node,
    uwb_name = uwb_measure,
    uwb_node,

    %% Prevent reusing same Hera measurements
    last_nav_seq = undefined,
    last_uwb_seq = undefined,

    %% Acceleration bias in the 2D plane, in m/s².
    %% IMPORTANT: these biases must match nav3 coordinates:
    %% nav3 Acc = [Ax, Ay, -Az] * 9.81
    acc_bias = {0.0, 0.0},

    %% Tuning
    sigma_acc = 1.0,    %% m/s², NAV/process uncertainty
    sigma_uwb = 0.10    %% m, UWB standard deviation
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEBUG
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-define(DEBUG, true).
-define(DEBUG_FILE, "uwb_nav_ekf_debug.log").

debug(Fmt, Args) ->
    case ?DEBUG of
        true ->
            Line = io_lib:format(
                "[~p] " ++ Fmt ++ "~n",
                [erlang:monotonic_time(millisecond) | Args]
            ),
            file:write_file(?DEBUG_FILE, Line, [append]);
        false ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Example init args:
%%
%% {
%%   {2.0, 1.0},                         %% initial position {Px0, Py0}, meters
%%   'sensor_fusion@nav_3',              %% node that publishes nav3
%%   'sensor_fusion@uwb_3',              %% node that publishes uwb_measure
%%   {-0.198, 0.404}                     %% acc bias in nav3 plane coordinates
%% }
%%
%% Note:
%% If your raw resting acceleration is:
%%   acc_y_mean_ms2 = -0.198
%%   acc_z_mean_ms2 = -0.404
%%
%% nav3 outputs third acceleration as -acc_z.
%% So the bias for nav3 plane coordinates is:
%%   {acc_y_mean_ms2, -acc_z_mean_ms2}
%%
%% Example:
%%   {-0.198, 0.404}
%%
% init({{Px0, Py0}, NavNode, UwbNode, AccBias}) ->
%     init({{Px0, Py0}, NavNode, UwbNode, AccBias, 1.0, 0.10});
init({{Px0, Py0}, NavNode, UwbNode, AccBias}) ->
    init({{Px0, Py0}, NavNode, UwbNode, AccBias, 2, 0.2});

init({{Px0, Py0}, NavNode, UwbNode, AccBias, SigmaAcc, SigmaUwb}) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 4
    },

    X0 = mat:matrix([
        [Px0],
        [Py0],
        [0.0],
        [0.0]
    ]),

    %% Position initially known reasonably well, velocity less known.
    % P0 = mat:diag([
    %     0.05,
    %     0.05,
    %     1.0,
    %     1.0
    % ]),
    P0 = mat:diag([
        0.0025,
        0.0025,
        0.10,
        0.10
    ]),

    State = #state{
        x = X0,
        p = P0,
        nav_node = NavNode,
        uwb_node = UwbNode,
        acc_bias = AccBias,
        sigma_acc = SigmaAcc,
        sigma_uwb = SigmaUwb
    },

    debug("INIT", []),

    {ok, State, Spec}.


measure(State0) ->
    % debug("try_predict_with_nav", []),
    %% 1. Predict whenever a new NAV sample is available.
    {State1, DidPredict} = try_predict_with_nav(State0),
    % debug("ok", []),

    % debug("try_update_with_uwb", []),
    %% 2. Correct whenever a new UWB sample is available.
    {State2, DidUpdate} = try_update_with_uwb(State1),
    % debug("ok", []),

    case DidPredict orelse DidUpdate of
        true ->
            % debug("State (~p, ~p)", [DidPredict, DidUpdate]),
            {Px, Py, Vx, Vy} = state_to_tuple(State2#state.x),
            {ok, [Px, Py, Vx, Vy], State2};

        false ->
            % debug("State (~p, ~p)", [DidPredict, DidUpdate]),
            {undefined, State2}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NAV prediction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try_predict_with_nav(State = #state{
    nav_name = NavName,
    nav_node = NavNode,
    last_nav_seq = LastSeq
}) ->
    case hera_data:get(NavName, NavNode) of
        [{_Node, Seq, _Timestamp, Data}] ->
            % debug("hera_data:get(~p, ~p): TRUE", [NavName, NavNode]), 
            case is_new_seq(Seq, LastSeq) of
                true ->
                    % debug("is_new_seq(~p, ~p): TRUE", [Seq, LastSeq]),
                    % NowUs = erlang:monotonic_time(microsecond),
                    % predict_from_nav_sample(Seq, NowUs, Data, State);
                    predict_from_nav_sample(Seq, _Timestamp, Data, State);

                false ->
                    % debug("is_new_seq(~p, ~p): FALSE", [Seq, LastSeq]),
                    {State, false}
            end;
        _ ->
            % debug("hera_data:get(~p, ~p): FALSE", [NavName, NavNode]),
            {State, false}
    end.


predict_from_nav_sample(Seq, NowMs, Data, State = #state{
    last_nav_t_us = undefined
}) ->
    %% First NAV sample: initialize timing, but do not predict yet.
    %% Otherwise the first dt could be arbitrary.
    % debug("PREDICT_from_nav_sample(undefined)", []),
    _ = Data,
    {State#state{last_nav_t_us = NowMs, last_nav_seq = Seq}, false};

predict_from_nav_sample(Seq, NowMs, Data, State = #state{
    x = X0,
    p = P0,
    last_nav_t_us = LastMs,
    acc_bias = {BiasX, BiasY},
    sigma_acc = SigmaAcc
}) ->
    %% nav3 data:
    %% [Ax, Ay, -Az, Gx, Gy, Gz, Mx, My, Mz]
    %% Acceleration is already in m/s².
    %%
    %% Since your board has X vertical, the horizontal plane is nav3 axes 2 and 3.
    % debug("PREDICT_from_nav_sample(true)", []),
    [_AccVertical, APlaneXRaw, APlaneYRaw | _] = Data,
    % debug("PREDICT_from_nav_sample(true) 1", []),
    % Dt0 = (NowUs - LastUs) / 1000000.0,
    Dt0 = (NowMs - LastMs) / 1000.0,
    % debug("PREDICT_from_nav_sample(true) 2", []),
    Dt = clamp_dt(Dt0),
    % debug("PREDICT_from_nav_sample(true) 3", []),
    APlaneX = APlaneXRaw - BiasX,
    % debug("PREDICT_from_nav_sample(true) 4", []),
    APlaneY = APlaneYRaw - BiasY,
    % debug("PREDICT_from_nav_sample(true) 5", []),

    % debug("nav_prediction_model", []),
    {FFun, JFFun, Q} = nav_prediction_model(Dt, APlaneX, APlaneY, SigmaAcc),
    % debug("EKF_predict", []),
    {X1, P1} = hera2:ekf_predict({X0, P0}, FFun, JFFun, Q),
    % debug("should return true", []),
    {State#state{x = X1, p = P1, last_nav_t_us = NowMs, last_nav_seq = Seq}, true}.


nav_prediction_model(Dt, Ax, Ay, SigmaAcc) ->
    Dt2 = Dt * Dt,

    Damp = math:exp(-Dt / 1.0),

    F = mat:matrix([
        [1.0, 0.0, Dt,   0.0],
        [0.0, 1.0, 0.0,  Dt ],
        [0.0, 0.0, Damp, 0.0],
        [0.0, 0.0, 0.0,  Damp]
    ]),

    B = mat:matrix([
        [0.5 * Dt2, 0.0],
        [0.0,       0.5 * Dt2],
        [Dt,        0.0],
        [0.0,       Dt]
    ]),

    U = mat:matrix([
        [Ax],
        [Ay]
    ]),

    %% Xp = F X + B U
    FFun = fun(X) ->
        mat:'+'(mat:'*'(F, X),mat:'*'(B, U))
    end,

    %% Jacobian of f(X) = F X + B U is F.
    JFFun = fun(_X) ->
        F
    end,

    %% More physical process noise:
    %% Q = B * Qa * B^T
    %% where Qa is acceleration uncertainty.
    Sa2 = SigmaAcc * SigmaAcc,
    Qa = mat:diag([Sa2, Sa2]),
    Q = mat:eval([B, '*', Qa, '*´', B]),

    {FFun, JFFun, Q}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UWB update, one anchor measurement at a time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try_update_with_uwb(State = #state{
    uwb_name = UwbName,
    uwb_node = UwbNode,
    last_uwb_seq = LastSeq
}) ->
    case hera_data:get(UwbName, UwbNode) of
        [{_Node, Seq, _Timestamp, Data}] ->
            case is_new_seq(Seq, LastSeq) of
                true ->
                    % debug("uwb,~p,~p,~p,~p", [Seq, _Timestamp, Data]),
                    update_from_uwb_sample(Seq, Data, State);

                false ->
                    {State, false}
            end;
        _ ->
            {State, false}
    end.


update_from_uwb_sample(Seq, Data, State = #state{
    x = X0,
    p = P0,
    sigma_uwb = SigmaUwb
}) ->
    %% Expected UWB data:
    %% [AnchorId, DistanceCm, AnchorX, AnchorY]
    %%
    %% Example CSV/log:
    %% seq,timestamp,anchorID,distance_cm,x,y
    %% 143,-576460232362,1,202.21,0.0,0.0
    [AnchorId, DistanceCm, AnchorX, AnchorY] = Data,
    _ = AnchorId,
    DistanceM = DistanceCm / 100.0,

    {HFun, JHFun, R, Z} =
        uwb_range_measurement_model(DistanceM, AnchorX, AnchorY, SigmaUwb),
    {X1, P1} = hera2:ekf_update({X0, P0}, HFun, JHFun, R, Z),
    {State#state{x = X1, p = P1, last_uwb_seq = Seq}, true}.


uwb_range_measurement_model(DistanceM, AnchorX, AnchorY, SigmaUwb) ->
    %% h(X) = sqrt((px-anchor_x)^2 + (py-anchor_y)^2)
    HFun = fun(X) ->
        {RPred, _Dx, _Dy} = predicted_range(X, AnchorX, AnchorY),
        mat:matrix([[RPred]])
    end,

    %% Jacobian:
    %% H = [(px-anchor_x)/r, (py-anchor_y)/r, 0, 0]
    JHFun = fun(X) ->
        {RPred, Dx, Dy} = predicted_range(X, AnchorX, AnchorY),
        mat:matrix([ [Dx / RPred, Dy / RPred, 0.0, 0.0] ])
    end,
    R = mat:matrix([ [SigmaUwb * SigmaUwb] ]),
    Z = mat:matrix([ [DistanceM] ]),
    {HFun, JHFun, R, Z}.


predicted_range(X, AnchorX, AnchorY) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),
    Dx = Px - AnchorX,
    Dy = Py - AnchorY,
    R0 = math:sqrt(Dx * Dx + Dy * Dy),
    %% Avoid division by zero if estimate is exactly on the anchor.
    R = max(R0, 0.000001),
    {R, Dx, Dy}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_to_tuple(X) ->
    {
        mat:get(1, 1, X),
        mat:get(2, 1, X),
        mat:get(3, 1, X),
        mat:get(4, 1, X)
    }.


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