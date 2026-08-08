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
            {ok, [AnchorId, DistanceCm, X, Y], NextState};
            % T1 = erlang:monotonic_time(microsecond),
            % Dt = (T1 - T0) / 1000000.0,
            % {ok, [AnchorId, DistanceCm, X, Y, Dt], NextState};

        _ ->
            {undefined, NextState}
    end.