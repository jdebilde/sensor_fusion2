-module(uwb_measure).

-behaviour(hera_measure).

-export([
    init/1,
    measure/1
]).

-record(state, {
    anchors = [],
    current = 0
}).

%% init( {[{1, 0.0, 0.0}, {2, 1.0, 0.0}]} )
init({Anchors, Current}) ->
    Spec = #{
        name => ?MODULE,
        iter => infinity,
        % iter => 50,
        timeout => 0
    },
    State = #state{
        anchors = Anchors,
        current = Current
    },
    {ok, State, Spec}.

measure(State = #state{
    anchors = Anchors,
    current = Current
}) ->
    % T0 = erlang:monotonic_time(microsecond),
    N = length(Anchors),
    Index = (Current rem N) + 1,
    {AnchorId, X, Y} = lists:nth(Index, Anchors),

    NextState = State#state{current = Index},

    case uwb_tag:measure_distance(AnchorId) of
        {ok, DistanceCm, _} ->
            CorrectedDistanceCm = correct_distance(DistanceCm),
            {ok, [AnchorId, CorrectedDistanceCm, X, Y], NextState};
            % {ok, [AnchorId, DistanceCm, X, Y], NextState};

            % T1 = erlang:monotonic_time(microsecond),
            % Dt = (T1 - T0) / 1000000.0,
            % {ok, [AnchorId, DistanceCm, X, Y, Dt], NextState};

        _ ->
            {undefined, NextState}
    end.



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
