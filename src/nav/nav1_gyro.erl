-module(nav1_gyro).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    gyro,
    t0 = undefined
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
                "[nav1_gyro] [~p] " ++ Fmt ++ "~n",
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
    io:format("nav1_gyro (gyro): Place de pmod_nav flat and still!~n"),
    [Gx,Gy,Gz] = calibrate(acc, [out_x_g,out_y_g,out_z_g], 300),
    io:format("nav1_gyro (gyro): ~p,~p,~p [deg/s]~n", [Gx,Gy,Gz]),
    #cal{gyro=[Gx,Gy,Gz]}.

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

    [Gx,Gy,Gz] = pmod_nav:read(acc, [out_x_g,out_y_g,out_z_g]),
    %% gyro #{g_unit => dps} => dps
    [Gxx, Gyy, Gzz] = subtract([Gx, Gy, Gz], C#cal.gyro),
    Data = [T1, Dt, Gxx, Gyy, Gzz],
    C1 = C#cal{t0 = T1},
    {ok, Data, C1}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% scale(List, Factor) ->
%     [X*Factor || X <- List].


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].


subtract([X,Y,Z], [X0,Y0,Z0]) ->
    [X-X0, Y-Y0, Z-Z0].
