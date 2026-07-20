-module(uwb_position).

-export([
    bilateration/4,
    inside_bounds/2,
    choose_position/3,
    locate_2anchors/4
]).

%%% =========================================================
%%% Public API
%%% =========================================================

%% bilateration({X1,Y1}, {X2,Y2}, R1, R2) ->
%%     {ok, P1, P2} | {error, Reason}
%%
%% Positions in meters.
%% Radii in meters.
%%
%% Returns the two possible intersections of the circles.
bilateration({X1, Y1}, {X2, Y2}, R1, R2)
  when is_number(X1), is_number(Y1),
       is_number(X2), is_number(Y2),
       is_number(R1), is_number(R2),
       R1 >= 0, R2 >= 0 ->
    DX = X2 - X1,
    DY = Y2 - Y1,
    D = math:sqrt(DX * DX + DY * DY),

    case D of
        0.0 ->
            {error, same_anchor_position};

        _ ->
            case (D > R1 + R2) orelse (D < abs(R1 - R2)) of
                true ->
                    {error, no_intersection};

                false ->
                    A = (R1 * R1 - R2 * R2 + D * D) / (2.0 * D),
                    H2 = R1 * R1 - A * A,

                    %% H2 can become very slightly negative because of floating point errors
                    H =
                        case H2 < 0.0 of
                            true -> 0.0;
                            false -> math:sqrt(H2)
                        end,

                    Xm = X1 + A * DX / D,
                    Ym = Y1 + A * DY / D,

                    Rx = -DY * H / D,
                    Ry =  DX * H / D,

                    P1 = {Xm + Rx, Ym + Ry},
                    P2 = {Xm - Rx, Ym - Ry},

                    {ok, P1, P2}
            end
    end.

%% inside_bounds({X,Y}, {MinX,MinY,MaxX,MaxY}) -> boolean()
inside_bounds({X, Y}, {MinX, MinY, MaxX, MaxY})
  when is_number(X), is_number(Y),
       is_number(MinX), is_number(MinY),
       is_number(MaxX), is_number(MaxY) ->
    X >= MinX andalso X =< MaxX andalso
    Y >= MinY andalso Y =< MaxY.

%% choose_position(P1, P2, Bounds) ->
%%     {ok, Position}
%%   | {ambiguous, P1, P2}
%%   | {error, out_of_bounds}
choose_position(P1, P2, Bounds) ->
    case {inside_bounds(P1, Bounds), inside_bounds(P2, Bounds)} of
        {true, false} ->
            {ok, P1};

        {false, true} ->
            {ok, P2};

        {true, true} ->
            {ambiguous, P1, P2};

        {false, false} ->
            {error, out_of_bounds}
    end.

%% locate_2anchors({Anchor1Pos, D1}, {Anchor2Pos, D2}, Bounds, Unit) ->
%%     {ok, Position}
%%   | {ok, P1, P2}
%%   | {ambiguous, P1, P2}
%%   | {error, Reason}
%%
%% Anchor positions are in meters.
%% Distances can be in cm or m according to Unit.
%%
%% Unit = cm | m
locate_2anchors({Anchor1Pos, D1}, {Anchor2Pos, D2}, Bounds, Unit) ->
    R1 = normalize_distance(D1, Unit),
    R2 = normalize_distance(D2, Unit),

    case bilateration(Anchor1Pos, Anchor2Pos, R1, R2) of
        {ok, P1, P2} ->
            case choose_position(P1, P2, Bounds) of
                {ok, Pos} ->
                    {ok, Pos};

                {ambiguous, P1a, P2a} ->
                    {ambiguous, P1a, P2a};

                {error, out_of_bounds} ->
                    %% Return both points anyway, useful for debugging
                    {ok, P1, P2}
            end;

        Error ->
            Error
    end.

%%% =========================================================
%%% Internal helpers
%%% =========================================================

normalize_distance(D, cm) when is_number(D) ->
    D / 100.0;
normalize_distance(D, m) when is_number(D) ->
    D;
normalize_distance(D, Unit) ->
    erlang:error({unsupported_unit, Unit, D}).