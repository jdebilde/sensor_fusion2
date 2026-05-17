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
        iter => infinity
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
    % erlang:display({success, Anchors, Current}),
    N = length(Anchors),
    Index = (Current rem N) + 1,
    {AnchorId, X, Y} = lists:nth(Index, Anchors),

    NextState = State#state{current = Index},

    case uwb_tag:measure_distance(AnchorId) of
        {ok, DistanceCm, _} ->
            {ok, [AnchorId, DistanceCm, X, Y], NextState};

        _ ->
            {undefined, NextState}
    end.