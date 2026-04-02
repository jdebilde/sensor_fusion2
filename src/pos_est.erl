-module(pos_est).

%% Position estimator with reduced drift:
%% - reads raw IMU from nav3
%% - reads orientation quaternion from e11
%% - rotates acceleration into world frame
%% - subtracts gravity in world frame
%% - applies a simple zero-velocity update when stationary
%%
%% State:
%% [x, y, z, vx, vy, vz]^T

-behaviour(hera_measure).

-export([init/1, measure/1]).

-define(ACC_STILL_THR, 0.35).   % m/s^2
-define(GYRO_STILL_THR, 0.08).  % rad/s

init(_) ->
    X0 = mat:zeros(6,1),
    P0 = mat:eye(6),
    T0 = hera:timestamp(),
    Spec = #{
        name => ?MODULE,
        iter => infinity
    },
    {ok, {T0, X0, P0}, Spec}.

measure({T0, X0, P0}) ->
    T1 = hera:timestamp(),
    Dt = (T1 - T0) / 1000,

    DataNav = hera_data:get(nav3, node()),
    DataE11 = hera_data:get(e11, node()),

    case {DataNav, DataE11} of
        {[], _} ->
            {undefined, {T0, X0, P0}};
        {_, []} ->
            %% No orientation yet -> do nothing rather than integrating bad accel
            {undefined, {T0, X0, P0}};
        _ ->
            {_,_,_,NavData} = lists:last(DataNav),
            {_,_,_,QuatData} = lists:last(DataE11),

            [Ax, Ay, Az, Gx, Gy, Gz | _] = NavData,
            [Q0, Q1, Q2, Q3] = QuatData,

            %% Quaternion -> rotation matrix
            R = q2dcm([[Q0], [Q1], [Q2], [Q3]]),

            %% Rotate body acceleration into world frame
            %% Same idea as e13: Accrot = [Acc] * R^T ; RotAcc = Accrot - [0,0,-9.81]
            AccWorld0 = mat:'*'([[Ax, Ay, Az]], mat:tr(R)),
            AccWorld = mat:'-'(AccWorld0, [[0,0,-9.81]]),

            [[Awx, Awy, Awz]] = AccWorld,

            %% Simple stillness detection
            AccNorm = math:sqrt(Awx*Awx + Awy*Awy + Awz*Awz),
            GyroNorm = math:sqrt(Gx*Gx + Gy*Gy + Gz*Gz),

            %% Deadband on tiny accelerations
            Ax1 = deadband(Awx, 0.08),
            Ay1 = deadband(Awy, 0.08),
            Az1 = deadband(Awz, 0.08),

            A = mat:matrix([[Ax1],[Ay1],[Az1]]),
            F = make_F(Dt),
            B = make_B(Dt),
            Q = make_Q(Dt),

            Xp0 = mat:'+'(
                    mat:'*'(F, X0),
                    mat:'*'(B, A)
                  ),
            Pp = mat:'+'(
                    mat:'*'(F, mat:'*'(P0, mat:tr(F))),
                    Q
                 ),

            %% Zero-velocity update when stationary
            Xp = case is_stationary(AccNorm, GyroNorm) of
                true -> zero_velocity(Xp0);
                false -> Xp0
            end,

            {ok, mat:to_array(Xp), {T1, Xp, Pp}}
    end.

%%------------------------------------------------------------------------------
%% System matrices
%%------------------------------------------------------------------------------

make_F(Dt) ->
    mat:matrix([
        [1,0,0,Dt,0,0],
        [0,1,0,0,Dt,0],
        [0,0,1,0,0,Dt],
        [0,0,0,1,0,0],
        [0,0,0,0,1,0],
        [0,0,0,0,0,1]
    ]).

make_B(Dt) ->
    Dt2 = 0.5 * Dt * Dt,
    mat:matrix([
        [Dt2,0,0],
        [0,Dt2,0],
        [0,0,Dt2],
        [Dt,0,0],
        [0,Dt,0],
        [0,0,Dt]
    ]).

make_Q(Dt) ->
    %% Smaller than identity to avoid exploding covariance too fast
    S = 0.02 + 0.2*Dt,
    mat:diag([S,S,S,S,S,S]).

%%------------------------------------------------------------------------------
%% Helpers
%%------------------------------------------------------------------------------

deadband(X, Thr) when X > Thr -> X;
deadband(X, Thr) when X < -Thr -> X;
deadband(_, _) -> 0.0.

is_stationary(AccNorm, GyroNorm) ->
    AccNorm < ?ACC_STILL_THR andalso GyroNorm < ?GYRO_STILL_THR.

zero_velocity(X) ->
    [Px, Py, Pz, _, _, _] = mat:to_array(X),
    mat:matrix([[Px],[Py],[Pz],[0],[0],[0]]).

q2dcm([[Q0], [Q1], [Q2], [Q3]]) ->
    R00 = 2 * (Q0 * Q0 + Q1 * Q1) - 1,
    R01 = 2 * (Q1 * Q2 - Q0 * Q3),
    R02 = 2 * (Q1 * Q3 + Q0 * Q2),

    R10 = 2 * (Q1 * Q2 + Q0 * Q3),
    R11 = 2 * (Q0 * Q0 + Q2 * Q2) - 1,
    R12 = 2 * (Q2 * Q3 - Q0 * Q1),

    R20 = 2 * (Q1 * Q3 - Q0 * Q2),
    R21 = 2 * (Q2 * Q3 + Q0 * Q1),
    R22 = 2 * (Q0 * Q0 + Q3 * Q3) - 1,

    [[R00, R01, R02],
     [R10, R11, R12],
     [R20, R21, R22]].