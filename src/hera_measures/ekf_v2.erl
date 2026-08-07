-module(ekf_v2).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(state, {
    %% [m/s^2]
    g_nav = undefined,
    acc_bias = [0.0, 0.0, 0.0],
    gyro_bias = [0.0, 0.0, 0.0],
    t0 = undefined,
    yaw_deg = 0.0,
    pitch_deg = 0.0,
    roll_deg = 0.0,

    %% EKF state
    x,
    p,
    % last_nav_seq = undefined,
    stopped_count = 0
}).

-define(G, 9.80665).
-define(SIGMA_ACCEL_MEAS, 0.30).
-define(SIGMA_ACCEL_STATE, 0.20).

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
                "[ekf_v2] [~p] " ++ Fmt ++ "~n",
                [erlang:monotonic_time(millisecond) | Args]
            ),
            file:write_file(?DEBUG_FILE, Line, [append]);
        false ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("ekf_v2: Calibrating...~n"),
    io:format("ekf_v2 (acc): Keep the PmodNAV flat and still.~n"),
    [AxG, AyG, AzG] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("nav2 (acc): ~p,~p,~p [g]~n", [AxG, AyG, AzG]),
    [Ax, Ay, Az] = scale([AxG, AyG, AzG], ?G),
    Gnav = math:sqrt(Ax*Ax + Ay*Ay + Az*Az),
    {Roll_acc, Pitch_acc} = compute_roll_and_pitch(Ax, Ay, Az),
    io:format("ekf_v2: g = ~p [m/s^2]~n",[Gnav]),
    io:format("ekf_v2: roll = ~p [deg], pitch = ~p  [deg]~n",
        [Roll_acc, Pitch_acc]),

    io:format("ekf_v2 (gyro): Keep the PmodNAV flat and still.~n"),
    [Gx, Gy, Gz] = calibrate(acc, [out_x_g, out_y_g, out_z_g], 300),
    io:format("ekf_v2 (gyro bias): ~p, ~p, ~p [deg/s]~n", [Gx, Gy, Gz]),

    #state{
        g_nav = Gnav,
        acc_bias = [AxG, AyG, AzG],
        gyro_bias = [Gx, Gy, Gz],
        yaw_deg = 0.0,
        pitch_deg = Pitch_acc,
        roll_deg = Roll_acc
    }.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(State0) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 10
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

    T0 = erlang:monotonic_time(microsecond),

    {ok, State0#state{t0 = T0, x = X0, p = P0}, Spec}.

measure(State0 = #state{t0 = T0, yaw_deg = Yaw0, pitch_deg = Pitch0, roll_deg = Roll0, x = X0, p = P0}) ->
    T1 = erlang:monotonic_time(microsecond),

    Dt =
        case T0 of
            undefined -> 0.0;
            _ -> (T1 - T0) / 1000000.0
        end,

    %% 1. Read Ax, Ay, Az, Gx, Gy, Gz
    [AxG, AyG, AzG, GxRaw, GyRaw, GzRaw] =
        pmod_nav:read(acc, [
            out_x_xl, out_y_xl, out_z_xl,
            out_x_g,  out_y_g,  out_z_g
        ]),

    %% 2. Subtract gyro bias
    [Axb, Ayb, Azb] = subtract([AxG, AyG, AzG], State0#state.acc_bias),
    [Gx, Gy, Gz] = subtract([GxRaw, GyRaw, GzRaw], State0#state.gyro_bias),

    %% 3. Convert acc to m/s²
    [Ax, Ay, Az] = scale([AxG, AyG, AzG], ?G),
    [Ax_no_bias, Ay_no_bias, Az_no_bias] = scale([Axb, Ayb, Azb], ?G),

    %% 4. Apply gyro deadband
    Threshold = 0.1,
    GxDb = soft_deadband(Gx, Threshold),
    GyDb = soft_deadband(Gy, Threshold),
    GzDb = soft_deadband(Gz, Threshold),

    %% 6. Fusion roll/pitch with gyro using a complementary filter
    {RollDeg0, PitchDeg0} = compute_roll_and_pitch(Ax, Ay, Az),
    Alpha = 0.98,
    PitchDeg1 = wrap_180(Alpha * (Pitch0 + GyDb * Dt) + (1.0 - Alpha) * PitchDeg0),
    RollDeg1  = wrap_180(Alpha * (Roll0 + GzDb * Dt) + (1.0 - Alpha) * RollDeg0),

    %% 7. Integrate yaw with vertical gyro
    Yaw1 = wrap_180(Yaw0 + GxDb * Dt),

    %% 8. Estimate gravity from roll/pitch
    PitchRad = deg2rad(PitchDeg1),
    RollRad  = deg2rad(RollDeg1),
    YawRad   = deg2rad(Yaw1),
    % G = State0#state.g_nav,
    G = ?G,
    {GxEst, GyEst, GzEst} = estimate_gravity_rad(G, RollRad, PitchRad),

    %% 9. Subtract gravity
    % AxLin = soft_deadband(Ax - GxEst, Threshold),
    % AyLin = soft_deadband(Ay - GyEst, Threshold),
    % AzLin = soft_deadband(Az - GzEst, Threshold),
    AxLin = Ax - GxEst,
    AyLin = Ay - GyEst,
    AzLin = Az - GzEst,

    %% 10. Determine the plane's axes
    ABodyX = AyLin,
    ABodyY = AzLin,

    %% 11. Rotate using yaw
    AWorldX = ABodyX * math:cos(YawRad) - ABodyY * math:sin(YawRad),
    AWorldY = ABodyX * math:sin(YawRad) + ABodyY * math:cos(YawRad),

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
        [AWorldX],
        [AWorldY]
    ]),

    % Precision = 2,
    % Data = [T1, Dt, 
    %     round(RollDeg1, Precision), round(PitchDeg1, Precision), round(Yaw1, Precision),
    %     Gx, Gy, Gz,
    %     Ax, Ay, Az,
    %     GxEst, GyEst, GzEst,
    %     AxLin, AyLin, AzLin,
    %     AWorldX, AWorldY
    % ],

    {XUpdate, P1} = kalman:kf_update({XPred, PPred}, H, R, Z),

    % Stopped0 = is_stopped(AWorldX, AWorldY, Gx, Gy, Gz),
    Stopped0 = is_stopped(Ax_no_bias, Ay_no_bias, Az_no_bias, Gx, Gy, Gz),

    {StateAfterStopCount, Stopped} = update_stopped_count(State0, Stopped0),

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
        p = P1,
        t0 = T1,
        yaw_deg = Yaw1,
        pitch_deg = PitchDeg1,
        roll_deg = RollDeg1
    },

    [Spx, Spy, Svx, Svy, Sax, Say] = state_to_list(X1),

    {ok, [Spx, Spy, Svx, Svy, Sax, Say, StoppedInt], State1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scale(List, Factor) ->
    [X*Factor || X <- List].

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

wrap_180(A) when A > 180.0  -> wrap_180(A - 360.0);
wrap_180(A) when A =< -180.0 -> wrap_180(A + 360.0);
wrap_180(A)                 -> A.

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

soft_deadband(V, T) when V > T  -> V - T;
soft_deadband(V, T) when V < -T -> V + T;
soft_deadband(_V, _T)           -> 0.0.

%% Calculates pitch and roll with the vertical X-axis at rest.
%%     - ax: Vertical (up/down)
%%     - ay: Lateral (left/right)
%%     - az: Longitudinal (forward/backward)
compute_roll_and_pitch(Ax, Ay, Az) ->
    ToDeg = 180 / math:pi(),
    %% Roll: left/right tilt
    %% Gravity is projected onto the Y-axis.
    Roll = math:atan2(Ay, Ax) * ToDeg,
    %% Pitch: forward/backward tilt
    %% Gravity is projected onto the Z-axis.
    Pitch = math:atan2(-Az, math:sqrt(Ax*Ax + Ay*Ay)) * ToDeg,
    {Roll, Pitch}.

estimate_gravity_rad(G, RollRad, PitchRad) ->
    GxEst = G * math:cos(PitchRad) * math:cos(RollRad),
    GyEst = G * math:cos(PitchRad) * math:sin(RollRad),
    GzEst = -G * math:sin(PitchRad),
    {GxEst, GyEst, GzEst}.

deg2rad(Deg) ->
    Deg * math:pi() / 180.0.

state_to_list(X) ->
    [
        mat:get(1, 1, X),
        mat:get(2, 1, X),
        mat:get(3, 1, X),
        mat:get(4, 1, X),
        mat:get(5, 1, X),
        mat:get(6, 1, X)
    ].

% is_stopped(AWorldX, AWorldY, Gx, Gy, Gz) ->
%     AccNorm = math:sqrt(AWorldX*AWorldX + AWorldY*AWorldY),
%     GyroNorm = math:sqrt(Gx*Gx + Gy*Gy + Gz*Gz),

%     AccNorm =< ?ACC_STOP_THRESHOLD andalso
%     GyroNorm =< ?GYRO_STOP_THRESHOLD.
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
