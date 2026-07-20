-module(e_nav_xytheta_kf).

-behaviour(hera_measure).

-export([calibrate/0, init/1, measure/1]).

-define(G, 9.81).
-define(PI, 3.141592653589793).
-define(TWO_PI, 6.283185307179586).

-record(cal, {
    acc_bias,
    gyro_bias,
    mag_bias,
    theta0
}).

-record(st, {
    t,
    x,
    p,
    cal
}).

calibrate() ->
    io:format("Place la board immobile, à plat, orientation theta=0.~n"),
    [Ax,Ay,Az] = avg(acc, [out_x_xl,out_y_xl,out_z_xl], 500),
    [Gx,Gy,Gz] = avg(acc, [out_x_g,out_y_g,out_z_g], 500),

    io:format("Calibration mag simple: tourne la board en yaw 180°, puis enter.~n"),
    [Mx1,My1,Mz1] = avg(mag, [out_x_m,out_y_m,out_z_m], 50),
    _ = io:get_line("Enter après rotation 180°: "),
    [Mx2,My2,Mz2] = avg(mag, [out_x_m,out_y_m,out_z_m], 50),

    MBx = 0.5*(Mx1+Mx2),
    MBy = 0.5*(My1+My2),
    MBz = 0.5*(Mz1+Mz2),

    Theta0 = heading(Mx1-MBx, My1-MBy),

    #cal{
        acc_bias = {Ax,Ay,Az},
        gyro_bias = {Gx,Gy,Gz},
        mag_bias = {MBx,MBy,MBz},
        theta0 = Theta0
    }.

init(Cal) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 10
    },

    %% x = [x, y, theta, vx, vy, omega]
    X0 = mat:matrix([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]),
    P0 = mat:diag([0.01,0.01,0.05, 0.5,0.5,0.2]),

    {ok, #st{t=hera:timestamp(), x=X0, p=P0, cal=Cal}, Spec}.

measure(St=#st{t=T0, x=X0, p=P0, cal=Cal}) ->
    T1 = hera:timestamp(),
    Dt0 = (T1 - T0) / 1000,
    Dt = clamp(Dt0, 0.001, 0.2),

    {AxB, AyB, Gz, ThetaMag0} = read_nav(Cal),

    [X,Y,Theta,Vx,Vy,Omega] = mat:to_array(X0),

    %% Accélération body -> world
    C = math:cos(Theta),
    S = math:sin(Theta),
    AxW = C*AxB - S*AyB,
    AyW = S*AxB + C*AyB,

    %% Prediction inertielle
    XpList = [
        X + Vx*Dt + 0.5*AxW*Dt*Dt,
        Y + Vy*Dt + 0.5*AyW*Dt*Dt,
        wrap(Theta + Omega*Dt),
        Vx + AxW*Dt,
        Vy + AyW*Dt,
        Omega
    ],
    Xp = mat:matrix([[V] || V <- XpList]),

    F = mat:matrix([
        [1,0,0,Dt,0,0],
        [0,1,0,0,Dt,0],
        [0,0,1,0,0,Dt],
        [0,0,0,1,0,0],
        [0,0,0,0,1,0],
        [0,0,0,0,0,1]
    ]),

    Q = mat:diag([0.002,0.002,0.0005, 0.20,0.20,0.01]),
    Pp = mat:'+'(mat:'*'(mat:'*'(F, P0), mat:tr(F)), Q),

    %% Correction avec cap magnétomètre + vitesse gyro z
    ThetaMag = unwrap_near(ThetaMag0, lists:nth(3, XpList)),

    H = mat:matrix([
        [0,0,1,0,0,0],
        [0,0,0,0,0,1]
    ]),
    Z = mat:matrix([[ThetaMag],[Gz]]),
    R = mat:diag([0.10, 0.02]),

    {X1Raw, P1} = kalman:kf_update({Xp, Pp}, H, R, Z),
    [X1,Y1,Theta1,Vx1,Vy1,Omega1] = mat:to_array(X1Raw),

    Values = [X1, Y1, wrap(Theta1), Vx1, Vy1, Omega1],
    X1Fixed = mat:matrix([[V] || V <- Values]),

    {ok, Values, St#st{t=T1, x=X1Fixed, p=P1}}.

read_nav(#cal{
    acc_bias={ABx,ABy,_ABz},
    gyro_bias={_GBx,_GBy,GBz},
    mag_bias={MBx,MBy,_MBz},
    theta0=Theta0
}) ->
    [Ax,Ay,_Az,Gx,Gy,GzRaw] = pmod_nav:read(acc, [
        out_x_xl,out_y_xl,out_z_xl,
        out_x_g,out_y_g,out_z_g
    ]),
    [Mx,My,_Mz] = pmod_nav:read(mag, [out_x_m,out_y_m,out_z_m]),

    %% En m/s², gravité retirée par calibration à plat.
    AxB = (Ax - ABx) * ?G,
    AyB = (Ay - ABy) * ?G,

    %% rad/s
    Gz = -(GzRaw - GBz) * math:pi() / 180,

    ThetaMag = wrap(heading(Mx-MBx, My-MBy) - Theta0),

    {AxB, AyB, Gz, ThetaMag}.

heading(Mx, My) ->
    math:atan2(My, Mx).

avg(Comp, Regs, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Regs)) || _ <- lists:seq(1,N)],
    {X,Y,Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].

clamp(V, Min, Max) when V < Min -> Min;
clamp(V, Min, Max) when V > Max -> Max;
clamp(V, _Min, _Max) -> V.

wrap(A) when A > ?PI ->
    wrap(A - ?TWO_PI);
wrap(A) when A < -?PI ->
    wrap(A + ?TWO_PI);
wrap(A) ->
    A.

unwrap_near(A, Ref) ->
    wrap(A - Ref) + Ref.