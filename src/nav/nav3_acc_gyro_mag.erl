-module(nav3_acc_gyro_mag).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    acc,
    gyro,
    t0 = undefined
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("nav3_acc_gyro_mag (acc): Calibrating... Do not move the pmod_nav!!~n"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("nav3_acc_gyro_mag (acc): ~p,~p,~p [g]~n", [Ax,Ay,Az]),
    io:format("nav3_acc_gyro_mag (gyro): Place de pmod_nav flat and still!~n"),
    [Gx,Gy,Gz] = calibrate(acc, [out_x_g,out_y_g,out_z_g], 300),
    io:format("nav3_acc_gyro_mag (gyro): ~p,~p,~p [deg/s]~n", [Gx,Gy,Gz]),
    #cal{acc=[Ax,Ay,Az], gyro=[Gx,Gy,Gz]}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(C) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 10
    },
    T0 = erlang:monotonic_time(microsecond),
    C1 = C#cal{t0 = T0},
    {ok, C1, Spec}.


measure(C = #cal{t0 = T0}) ->
    T1 = erlang:monotonic_time(microsecond),

    Dt =
        case T0 of
            undefined ->
                0.014;
            _ ->
                (T1 - T0) / 1000000.0
        end,

    [Ax,Ay,Az, Gx,Gy,Gz] = pmod_nav:read(acc, [
        out_x_xl,out_y_xl,out_z_xl,
        out_x_g,out_y_g,out_z_g]),
    Acc = subtract([Ax, Ay, Az], C#cal.acc),
    Gyro = subtract([Gx, Gy, Gz], C#cal.gyro),
    %% acc #{xl_unit => g} => m/s^2
    [Axx, Ayy, Azz] = scale(Acc, 9.81),
    %% gyro #{g_unit => dps} => dps
    [Gxx, Gyy, Gzz] = Gyro,
    Data = [T1, Dt, Axx, Ayy, Azz, Gxx, Gyy, Gzz],
    C1 = C#cal{t0 = T1},
    {ok, Data, C1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scale(List, Factor) ->
    [X*Factor || X <- List].


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].


subtract([X,Y,Z], [X0,Y0,Z0]) ->
    [X-X0, Y-Y0, Z-Z0].
