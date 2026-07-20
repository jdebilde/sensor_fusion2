-module(hera2).

-export([
    ekf_predict/4,
    ekf_update/5
]).

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

%% Extended Kalman prediction step.
%%
%% X0 : state vector matrix
%% P0 : covariance matrix
%% FFun : function that predicts the next state, FFun(X0) -> Xp
%% JFFun : function that returns the Jacobian of F at X0, JFFun(X0) -> Jf
%% Q : process noise covariance
%%
%% Returns:
%% {Xp, Pp}
%%
ekf_predict({X0, P0}, FFun, JFFun, Q) ->
    Xp = FFun(X0),
    Jf = JFFun(X0),
    Pp = mat:eval([Jf, '*', P0, '*´', Jf, '+', Q]),
    {Xp, Pp}.


%% Extended Kalman update step.
%%
%% Xp : predicted state vector matrix
%% Pp : predicted covariance matrix
%% HFun : function that predicts the measurement, HFun(Xp) -> ZPred
%% JHFun : function that returns the Jacobian of H at Xp, JHFun(Xp) -> Jh
%% R : measurement noise covariance
%% Z : real measurement
%%
%% Returns:
%% {X1, P1}
%%
ekf_update({Xp, Pp}, HFun, JHFun, R, Z) ->
    Jh = JHFun(Xp),

    %% Innovation covariance:
    %% S = H P H^T + R
    S = mat:eval([Jh, '*', Pp, '*´', Jh, '+', R]),

    %% Kalman gain:
    %% K = P H^T S^-1
    K = mat:eval([Pp, '*´', Jh, '*', mat:inv(S)]),

    %% Innovation:
    %% Y = Z - h(X)
    Y = mat:'-'(Z, HFun(Xp)),

    %% State correction:
    %% X1 = Xp + K Y
    X1 = mat:eval([K, '*', Y, '+', Xp]),

    %% Covariance correction:
    %% P1 = Pp - K H Pp
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jh, '*', Pp])),

    {X1, P1}.