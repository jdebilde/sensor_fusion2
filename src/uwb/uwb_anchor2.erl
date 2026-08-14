-module(uwb_anchor2).

-export([
    start/1,
    start_link/1,
    stop/0,
    loop/1,
    print_delay/0
]).

-define(SERVER, ?MODULE).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TX_ANTD, 16415).
-define(RX_ANTD, 16415).

-define(TS_MASK, 16#FFFFFFFFFF).
-define(TS_WRAP, 16#10000000000).

-include_lib("grisp/include/pmod_uwb.hrl").

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

configure_uwb() ->
    pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    pmod_uwb:set_frame_timeout(16#FFFF).

ensure_started() ->
    case whereis(pmod_uwb) of
        undefined ->
            case pmod_uwb:start_link(spi2, []) of
                {ok, _Pid} ->
                    configure_uwb(),
                    ok;
                {error, {already_started, _Pid}} ->
                    configure_uwb(),
                    ok;
                Other ->
                    Other
            end;
        _Pid ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PUBLIC API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(AnchorId) ->
    case whereis(?SERVER) of
        undefined ->
            Pid = spawn(?MODULE, start_link, [AnchorId]),
            {ok, Pid};

        Pid ->
            {error, {already_started, Pid}}
    end.

start_link(AnchorId)
  when is_integer(AnchorId),
       AnchorId >= 0,
       AnchorId =< 255 ->
    register(?SERVER, self()),

    case ensure_started() of
        ok ->
            [grisp_led:color(L, green) || L <- [1, 2]],
            loop(AnchorId);

        Error ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            unregister(?SERVER),
            exit({uwb_start_failed, Error})
    end.

stop() ->
    case whereis(?SERVER) of
        undefined ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            ok;

        Pid ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            Pid ! stop,
            ok
    end.

print_delay() ->
    io:format("TX_ANTD=~p RX_ANTD=~p ~n", [?TX_ANTD, ?RX_ANTD]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

loop(AnchorId) ->
    receive
        stop ->
            ok
    after 0 ->
        case pmod_uwb:reception() of
            {_, <<"POLL:", Seq:8, AnchorId:8>>} ->
                handle_poll(AnchorId, Seq);

            _ ->
                ok
        end,
        loop(AnchorId)
    end.

handle_poll(AnchorId, Seq) ->
    %% ---- T2: receive POLL ----
    #{rx_stamp := T2_0} = pmod_uwb:read(rx_time),
    T2 = ts_norm(T2_0),

    %% ---- T3: immediate RESP ----
    %% <<"RESP:", Seq:8, AnchorId:8, T2:40>>
    Resp = <<"RESP:", Seq:8, AnchorId:8, T2:40>>,
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T3_0} = pmod_uwb:read(tx_time),
    T3 = ts_norm(T3_0),

    %% ---- WAIT FINAL ----
    case pmod_uwb:reception() of
        {_, <<"FINAL:", Seq:8, AnchorId:8, T1_0:40, T4_0:40, T5_0:40>>} ->
            T1 = ts_norm(T1_0),
            T4 = ts_norm(T4_0),
            T5 = ts_norm(T5_0),

            %% ---- T6: receive FINAL ----
            #{rx_stamp := T6_0} = pmod_uwb:read(rx_time),
            T6 = ts_norm(T6_0),

            %% ---- DS-TWR ----
            Tround1 = ts_sub(T4, T1),
            Treply1 = ts_sub(T3, T2),
            Tround2 = ts_sub(T6, T3),
            Treply2 = ts_sub(T5, T4),
            % io:format(
            %     "T2=~p T3=~p diff=~p Delay=~.2f us~n",
            %     [T2, T3, Treply1, Treply1 * ?DWT_TIME_UNIT * 1000.0]
            % ),

            Den = Tround1 + Tround2 + Treply1 + Treply2,

            case Den of
                0 ->
                    error;
                _ ->
                    ToF =
                        ((Tround1 * Tround2) - (Treply1 * Treply2)) div Den,

                    DistanceM = ToF * ?DWT_TIME_UNIT * ?C,
                    DistanceCm = DistanceM * 100.0,

                    %% ---- REPORT ----
                    %% We encode the distance as an integer to avoid floating-point numbers.
                    %% Ex: 93.41 cm -> 9341
                    DistanceCmInt = round(DistanceCm * 100.0),
                    Report = <<"REPORT:", Seq:8, AnchorId:8, DistanceCmInt:32/signed>>,

                    pmod_uwb:transmit(Report),

                    % io:format(
                    %     "Anchor ~p seq=~p distance=~.2f cm~n",
                    %     [AnchorId, Seq, DistanceCm]
                    % ),
                    ok
            end;

        {_, <<"FINAL:", Seq:8, OtherAnchorId:8, _/binary>>} ->
            ok;

        {error, Reason} ->
            error;

        _ ->
            error
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% HELPERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts_norm(T) ->
    T band ?TS_MASK.

ts_sub(Newer, Older) when Newer >= Older ->
    Newer - Older;
ts_sub(Newer, Older) ->
    (Newer + ?TS_WRAP) - Older.