-module(nav1_mag).

-behaviour(hera_measure).

-export([calibrate/0]).
-export([init/1, measure/1]).

-record(cal, {
    mag,
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
                "[nav1_mag] [~p] " ++ Fmt ++ "~n",
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
    %% Assumption: The PmodNAV X axis is vertical.
    %% x -> Z: Yaw
    %% y -> X: Pitch
    %% z -> Y: Roll
    io:format("nav1_mag (mag): Place de pmod_nav flat and still!~n"),
    [Mx1,My1,Mz1] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    _ = io:get_line("nav1_mag (mag): Turn the pmod_nav 180° around the z axis then press enter"),
    [_,My2,Mz2] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    _ = io:get_line("nav1_mag (mag): Turn the pmod_nav 180° around the x axis then press enter"),
    [Mx2,_,_] = calibrate(mag, [out_x_m, out_y_m, out_z_m], 10),
    BiasX = 0.5*(Mx1+Mx2),
    BiasY = 0.5*(My1+My2),
    BiasZ = 0.5*(Mz1+Mz2),
    io:format("nav1_mag (mag): BiasX = ~p~n", [BiasX]),
    io:format("nav1_mag (mag): BiasY = ~p~n", [BiasY]),
    io:format("nav1_mag (mag): BiasZ = ~p~n", [BiasZ]),
    #cal{mag=[BiasX,BiasY,BiasZ]}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(C) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 0
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

    [Mx, My, Mz] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    [Mxx, Myy, Mzz] = subtract([Mx, My, Mz], C#cal.mag),
    Data = [T1, Dt, Mxx, Myy, Mzz],
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
