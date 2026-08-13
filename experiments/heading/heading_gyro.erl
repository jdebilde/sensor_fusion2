-module(heading_gyro).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    acc = [0.0, 0.0, 0.0],
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
    io:format("heading_gyro: Calibrating...~n"),
    io:format("heading_gyro (acc): Keep the PmodNAV flat and still.~n"),
    [Ax, Ay, Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("heading_gyro (acc bias): ~p, ~p, ~p [g]~n", [Ax, Ay, Az]),

    io:format("heading_gyro (gyro): Keep the PmodNAV flat and still.~n"),
    [Gx, Gy, Gz] = calibrate(acc, [out_x_g, out_y_g, out_z_g], 300),
    io:format("heading_gyro (gyro bias): ~p, ~p, ~p [deg/s]~n", [Gx, Gy, Gz]),

    #cal{
        acc = [Ax, Ay, Az],
        gyro = [Gx, Gy, Gz],
        yaw_deg = 0.0,
        pitch_deg = 0.0,
        roll_deg = 0.0
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

    [Ax, Ay, Az, GxRaw, GyRaw, GzRaw] =
        pmod_nav:read(acc, [
            out_x_xl, out_y_xl, out_z_xl,
            out_x_g,  out_y_g,  out_z_g
        ]),

    [Gx, Gy, Gz] = subtract([GxRaw, GyRaw, GzRaw], C#cal.gyro),

    %% Applique le deadband sur chaque axe proprement
    Threshold = 0.1,
    GxDb = soft_deadband(Gx, Threshold),
    GyDb = soft_deadband(Gy, Threshold),
    GzDb = soft_deadband(Gz, Threshold),

    %% 1. Calcul de l'orientation brute via l'accéléromètre
    {Roll_acc, Pitch_acc} = compute_roll_and_pitch(Ax, Ay, Az),

    %% 2. Détection de l'état "à l'arrêt"
    %% Si la vitesse de rotation sur le Pitch (Gy) et le Roll (Gz) est quasi nulle
    % IsAtRest = (abs(GyDb) < 0.02) andalso (abs(GzDb) < 0.02),
    IsAtRest = false,

    %% 3. Filtre complémentaire adaptatif
    %% Alpha représente la confiance accordée au gyroscope.
    %% En mouvement : 98% Gyro / 2% Accel (filtre les accélérations latérales).
    %% À l'arrêt   : 0% Gyro / 100% Accel (recale immédiatement l'assiette sur la gravité).
    Alpha = case IsAtRest of
                true  -> 0.0; 
                false -> 0.99 
            end,

    %% Calcul des nouveaux angles fusionnés
    Pitch1 = wrap_180(Alpha * (Pitch0 + GyDb * Dt) + (1.0 - Alpha) * Pitch_acc),
    Roll1  = wrap_180(Alpha * (Roll0 + GzDb * Dt) + (1.0 - Alpha) * Roll_acc),

    %% Le Yaw reste en intégration pure (pour le moment)
    Yaw1 = wrap_180(Yaw0 + GxDb * Dt),

    Precision = 2,
    %% On renvoie les valeurs filtrées finales pour votre traitement
    Data = [T1, Dt, round(Roll1, Precision), round(Pitch1, Precision), round(Yaw1, Precision)],
    
    C1 = C#cal{t0 = T1, yaw_deg = Yaw1, pitch_deg = Pitch1, roll_deg = Roll1},

    {ok, Data, C1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% compute_roll_and_pitch(Ax, Ay, Az) ->
%     ToDeg = 180 / math:pi(),
%     % Roll (rotation around the Z-axis)
%     Roll  = math:atan2(Az, Ax) * ToDeg,
%     % Pitch (rotation around the Y-axis)
%     Pitch = math:atan2(Ay, math:sqrt(Ax*Ax + Az*Az)) * ToDeg,
%     {Roll, Pitch}.