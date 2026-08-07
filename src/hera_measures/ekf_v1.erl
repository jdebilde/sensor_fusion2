-module(ekf_v1).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-define(G, 9.80665).

-record(cal, {
    % [m/s^2]
    g_nav = undefined,
    gyro = [0.0, 0.0, 0.0],
    t0 = undefined,
    yaw_deg = 0.0,
    pitch_deg = 0.0,
    roll_deg = 0.0
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("ekf_v1: Calibrating...~n"),
    io:format("ekf_v1 (acc): Keep the PmodNAV flat and still.~n"),
    [AxG, AyG, AzG] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    [Ax, Ay, Az] = scale([AxG, AyG, AzG], ?G),
    Gnav = math:sqrt(Ax*Ax + Ay*Ay + Az*Az),
    {Roll_acc, Pitch_acc} = compute_roll_and_pitch(Ax, Ay, Az),
    io:format("ekf_v1: g = ~p [m/s^2]~n",[Gnav]),
    io:format("ekf_v1: roll = ~p [deg], pitch = ~p  [deg]~n",
        [Roll_acc, Pitch_acc]),

    io:format("ekf_v1 (gyro): Keep the PmodNAV flat and still.~n"),
    [Gx, Gy, Gz] = calibrate(acc, [out_x_g, out_y_g, out_z_g], 300),
    io:format("ekf_v1 (gyro bias): ~p, ~p, ~p [deg/s]~n", [Gx, Gy, Gz]),

    #cal{
        g_nav = Gnav,
        gyro = [Gx, Gy, Gz],
        yaw_deg = 0.0,
        pitch_deg = Pitch_acc,
        roll_deg = Roll_acc
    }.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(C) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 10
    },

    T0 = erlang:monotonic_time(microsecond),

    {ok, C#cal{t0 = T0}, Spec}.

measure(C = #cal{t0 = T0, yaw_deg = Yaw0, pitch_deg = Pitch0, roll_deg = Roll0}) ->
    T1 = erlang:monotonic_time(microsecond),

    Dt =
        case T0 of
            undefined -> 0.0;
            _ -> (T1 - T0) / 1000000.0
        end,

    % 1. Read Ax, Ay, Az, Gx, Gy, Gz
    [AxG, AyG, AzG, GxRaw, GyRaw, GzRaw] =
        pmod_nav:read(acc, [
            out_x_xl, out_y_xl, out_z_xl,
            out_x_g,  out_y_g,  out_z_g
        ]),

    % 2. Convert acc to m/s²
    [Ax, Ay, Az] = scale([AxG, AyG, AzG], ?G),

    % 3. Subtract gyro bias
    [Gx, Gy, Gz] = subtract([GxRaw, GyRaw, GzRaw], C#cal.gyro),

    % 4. Apply gyro deadband
    Threshold = 0.1,
    GxDb = soft_deadband(Gx, Threshold),
    GyDb = soft_deadband(Gy, Threshold),
    GzDb = soft_deadband(Gz, Threshold),

    % 6. Fusion roll/pitch with gyro using a complementary filter
    {RollDeg0, PitchDeg0} = compute_roll_and_pitch(Ax, Ay, Az),
    Alpha = 0.9 ,
    PitchDeg1 = wrap_180(Alpha * (Pitch0 + GyDb * Dt) + (1.0 - Alpha) * PitchDeg0),
    RollDeg1  = wrap_180(Alpha * (Roll0 + GzDb * Dt) + (1.0 - Alpha) * RollDeg0),

    % 7. Integrate yaw with vertical gyro
    Yaw1 = wrap_180(Yaw0 + GxDb * Dt),

    % 8. Estimate gravity from roll/pitch
    PitchRad = deg2rad(PitchDeg1),
    RollRad  = deg2rad(RollDeg1),
    YawRad   = deg2rad(Yaw1),
    G = C#cal.g_nav,
    {GxEst, GyEst, GzEst} = estimate_gravity_rad(G, RollRad, PitchRad),

    % 9. Subtract gravity
    AxLin = soft_deadband(Ax - GxEst, Threshold),
    AyLin = soft_deadband(Ay - GyEst, Threshold),
    AzLin = soft_deadband(Az - GzEst, Threshold),

    % 10. Determine the plane's axes
    ABodyX = AyLin,
    ABodyY = AzLin,

    % 11. Rotate using yaw
    AWorldX = ABodyX * math:cos(YawRad) - ABodyY * math:sin(YawRad),
    AWorldY = ABodyX * math:sin(YawRad) + ABodyY * math:cos(YawRad),

    Precision = 2,
    Data = [T1, Dt, 
        round(RollDeg1, Precision), round(PitchDeg1, Precision), round(Yaw1, Precision),
        Gx, Gy, Gz,
        Ax, Ay, Az,
        GxEst, GyEst, GzEst,
        AxLin, AyLin, AzLin,
        AWorldX, AWorldY
    ],

    C1 = C#cal{
        t0 = T1,
        yaw_deg = Yaw1,
        pitch_deg = PitchDeg1,
        roll_deg = RollDeg1
    },

    {ok, Data, C1}.

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

wrap_360(A) when A >= 360.0 -> wrap_360(A - 360.0);
wrap_360(A) when A < 0.0    -> wrap_360(A + 360.0);
wrap_360(A)                 -> A.

wrap_180(A) when A > 180.0  -> wrap_180(A - 360.0);
wrap_180(A) when A =< -180.0 -> wrap_180(A + 360.0);
wrap_180(A)                 -> A.

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

soft_deadband(V, T) when V > T  -> V - T;
soft_deadband(V, T) when V < -T -> V + T;
soft_deadband(_V, _T)           -> 0.0.

% Calculates pitch and roll with the vertical X-axis at rest.
%     - ax: Vertical (up/down)
%     - ay: Lateral (left/right)
%     - az: Longitudinal (forward/backward)
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
