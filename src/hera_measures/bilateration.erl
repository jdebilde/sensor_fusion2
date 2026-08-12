-module(bilateration).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    anchor1,
    anchor2,
    last_pos
}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API / Hera callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% bilateration:init({
%%     {1, 0.0, 0.0},
%%     {2, 2.0, 0.0},
%%     {0.9, 3.15}
%% }).
%%
%% anchor = {AnchorId, X, Y}
%% initial position = {Px0, Py0}

init({Anchor1, Anchor2, InitialPos}) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        % iter => 50,
        timeout => 0
    },

    State = #state{
        anchor1 = Anchor1,
        anchor2 = Anchor2,
        last_pos = InitialPos
    },

    {ok, State, Spec}.


measure(State0 = #state{
    anchor1 = {AnchorId1, X1, Y1},
    anchor2 = {AnchorId2, X2, Y2},
    last_pos = LastPos
}) ->
    case measure_two_anchors(AnchorId1, AnchorId2) of
        {ok, D1Cm, D2Cm} ->
            R1 = correct_distance(D1Cm) / 100.0,
            R2 = correct_distance(D2Cm) / 100.0,

            case bilaterate({X1, Y1}, R1, {X2, Y2}, R2) of
                {ok, PPlus, PMinus} ->
                    Pos = choose_closest(LastPos, PPlus, PMinus),
                    {Px, Py} = Pos,

                    State1 = State0#state{
                        last_pos = Pos
                    },
                    %% px, py, d1_m, d2_m, anchor1, anchor2, valid
                    {ok, [Px, Py, R1, R2, AnchorId1, AnchorId2, 1], State1};

                {error, Reason} ->
                    %% No valid intersection.
                    %% We keep the last position.
                    {Px, Py} = LastPos,
                    ReasonCode = reason_to_code(Reason),
                    {ok, [Px, Py, R1, R2, AnchorId1, AnchorId2, 0, ReasonCode], State0}
            end;

        {error, Reason} ->
            %% No valid measurement
            {Px, Py} = LastPos,
            ReasonCode = reason_to_code(Reason),
            {ok, [Px, Py, -1.0, -1.0, AnchorId1, AnchorId2, 0, ReasonCode], State0}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

measure_two_anchors(AnchorId1, AnchorId2) ->
    case try_measure_distance(AnchorId1) of
        {ok, D1Cm} ->
            case try_measure_distance(AnchorId2) of
                {ok, D2Cm} ->
                    {ok, D1Cm, D2Cm};

                {error, Reason2} ->
                    {error, Reason2}
            end;

        {error, Reason1} ->
            {error, Reason1}
    end.


try_measure_distance(AnchorId) ->
    case uwb_tag:measure_distance(AnchorId) of
        {ok, DistanceCm, _} ->
            case is_distance_cm_valid(DistanceCm) of
                true ->
                    {ok, DistanceCm};
                false ->
                    {error, bad_distance}
            end;

        _ ->
            {error, no_measure}
    end.


is_distance_cm_valid(DistanceCm) ->
    DistanceCm > 0.0 andalso DistanceCm < 1500.0.

bilaterate({X1, Y1}, R1, {X2, Y2}, R2) ->
    Dx = X2 - X1,
    Dy = Y2 - Y1,

    D = math:sqrt(Dx * Dx + Dy * Dy),

    case D =< 0.000001 of
        true ->
            {error, same_anchor_position};

        false ->
            %% No intersection: the circles are too far apart
            %% or one circle is entirely inside the other.
            NoIntersection = (D > R1 + R2) orelse (D < abs_float(R1 - R2)),

            case NoIntersection of
                true ->
                    {error, no_intersection};

                false ->
                    A = (R1 * R1 - R2 * R2 + D * D) / (2.0 * D),
                    %% H represents the perpendicular distance between the
                    %% anchor line and the two possible solutions
                    H2 = R1 * R1 - A * A,

                    case H2 < 0.0 of
                        true ->
                            {error, negative_h};

                        false ->
                            H = math:sqrt(H2),

                            %% unit vector pointing from anchor 1 to anchor 2
                            ExX = Dx / D,
                            ExY = Dy / D,

                            %% Compute where H is hitting on the on the
                            %% anchor line, to find the two possible solutions
                            Xm = X1 + A * ExX,
                            Ym = Y1 + A * ExY,

                            %% The the two possible solutions are perpendicular
                            PerpX = -ExY,
                            PerpY = ExX,

                            PPlus = {Xm + H * PerpX, Ym + H * PerpY},
                            PMinus = {Xm - H * PerpX, Ym - H * PerpY},

                            {ok, PPlus, PMinus}
                    end
            end
    end.

choose_closest(RefPos, P1, P2) ->
    D1 = dist2(RefPos, P1),
    D2 = dist2(RefPos, P2),

    case D1 =< D2 of
        true ->
            P1;
        false ->
            P2
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dist2({X1, Y1}, {X2, Y2}) ->
    Dx = X1 - X2,
    Dy = Y1 - Y2,
    Dx * Dx + Dy * Dy.

abs_float(X) when X < 0 ->
    -X;
abs_float(X) ->
    X.


reason_to_code(no_measure) ->
    1;
reason_to_code(bad_distance) ->
    2;
reason_to_code(same_anchor_position) ->
    3;
reason_to_code(no_intersection) ->
    4;
reason_to_code(negative_h) ->
    5;
reason_to_code(_) ->
    99.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% UWB bias correction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

correct_distance(D) ->
    Points = [
        {35.5572, 50.0},
        {93.2444, 100.0},
        {205.4238, 200.0},
        {308.4836, 300.0},
        {419.3858, 400.0},
        {515.4082, 500.0}
    ],
    interpolate(D, Points).

%% Below calibration range:
%% extrapolate using the first segment
interpolate(D, [{X1, Y1}, {X2, Y2} | _]) when D < X1 ->
    linear_interpolate(D, X1, Y1, X2, Y2);

%% Inside calibration range
interpolate(D, [{X1, Y1}, {X2, Y2} | _])
        when D >= X1, D =< X2 ->
    linear_interpolate(D, X1, Y1, X2, Y2);

%% Continue searching for the correct interval
interpolate(D, [_ | Rest]) when length(Rest) >= 2 ->
    interpolate(D, Rest);

%% Above calibration range:
%% extrapolate using the last segment
interpolate(D, [{X1, Y1}, {X2, Y2}]) ->
    linear_interpolate(D, X1, Y1, X2, Y2).

linear_interpolate(D, X1, Y1, X2, Y2) ->
    Y1 + (D - X1) * (Y2 - Y1) / (X2 - X1).