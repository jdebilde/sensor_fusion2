-module(nav1_acc).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    acc,
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
                "[nav1_acc] [~p] " ++ Fmt ++ "~n",
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
    io:format("nav1_acc (acc): Calibrating... Do not move the pmod_nav!!~n"),
    [Ax,Ay,Az] = calibrate(acc, [out_x_xl, out_y_xl, out_z_xl], 300),
    io:format("nav1_acc (acc): ~p,~p,~p [g]~n", [Ax,Ay,Az]),
    #cal{acc=[Ax,Ay,Az]}.

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

    [Ax,Ay,Az] = pmod_nav:read(acc, [out_x_xl,out_y_xl,out_z_xl]),
    %% acc #{xl_unit => g} => m/s^2
    [Axx, Ayy, Azz] = scale(subtract([Ax, Ay, Az], C#cal.acc), 9.81),
    Data = [T1, Dt, Axx, Ayy, Azz],
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
