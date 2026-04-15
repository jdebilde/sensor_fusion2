-module(uwb_anchor).

-export([start/1, loop/1]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TX_ANTD, 16453).
-define(RX_ANTD, 16453).

-define(TS_MASK, 16#FFFFFFFFFF).
-define(TS_WRAP, 16#10000000000).

-include_lib("grisp/include/pmod_uwb.hrl").

%%% =========================
%%% TIMESTAMP HELPERS
%%% =========================

ts_norm(T) ->
    T band ?TS_MASK.

ts_sub(Newer, Older) when Newer >= Older ->
    Newer - Older;
ts_sub(Newer, Older) ->
    (Newer + ?TS_WRAP) - Older.

%%% =========================
%%% INIT / CONFIG
%%% =========================

configure_uwb() ->
    pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    pmod_uwb:set_frame_timeout(16#FFFF).

%%% =========================
%%% PUBLIC API
%%% =========================

start(AnchorId) when is_integer(AnchorId), AnchorId >= 0, AnchorId =< 255 ->
    pmod_uwb:start_link(spi2, []),
    configure_uwb(),
    io:format("UWB anchor ~p started~n", [AnchorId]),
    loop(AnchorId).

%%% =========================
%%% MAIN LOOP
%%% =========================

loop(AnchorId) ->
    case pmod_uwb:reception() of
        %% POLL targeted to this anchor
        {_, <<"POLL:", Seq:8, AnchorId:8>>} ->
            handle_poll(AnchorId, Seq);

        %% POLL for another anchor -> ignore
        {_, <<"POLL:", Seq:8, OtherAnchorId:8>>} ->
            io:format(
                "Anchor ~p ignoring POLL seq=~p for anchor ~p~n",
                [AnchorId, Seq, OtherAnchorId]
            ),
            ok;

        {error, Reason} ->
            io:format("Anchor ~p reception error: ~p~n", [AnchorId, Reason]),
            ok;

        {_, Other} ->
            io:format("Anchor ~p unexpected frame: ~p~n", [AnchorId, Other]),
            ok;

        _ ->
            ok
    end,
    loop(AnchorId).

handle_poll(AnchorId, Seq) ->
    %% ---- T2: receive POLL ----
    #{rx_stamp := T2_0} = pmod_uwb:read(rx_time),
    T2 = ts_norm(T2_0),

    %% ---- T3: immediate RESP ----
    %% Payload:
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

            Den = Tround1 + Tround2 + Treply1 + Treply2,

            case Den of
                0 ->
                    io:format(
                        "Anchor ~p seq=~p invalid denominator~n",
                        [AnchorId, Seq]
                    ),
                    error;
                _ ->
                    ToF =
                        ((Tround1 * Tround2) - (Treply1 * Treply2)) div Den,

                    DistanceM = ToF * ?DWT_TIME_UNIT * ?C,
                    DistanceCm = DistanceM * 100.0,

                    io:format(
                        "Anchor ~p seq=~p distance=~.2f cm "
                        "(T1=~p T2=~p T3=~p T4=~p T5=~p T6=~p)~n",
                        [AnchorId, Seq, DistanceCm, T1, T2, T3, T4, T5, T6]
                    ),
                    ok
            end;

        {_, <<"FINAL:", Seq:8, OtherAnchorId:8, _/binary>>} ->
            io:format(
                "Anchor ~p seq=~p ignoring FINAL for anchor ~p~n",
                [AnchorId, Seq, OtherAnchorId]
            ),
            ok;

        {error, Reason} ->
            io:format(
                "Anchor ~p seq=~p FINAL error: ~p~n",
                [AnchorId, Seq, Reason]
            ),
            error;

        _ ->
            io:format(
                "Anchor ~p seq=~p missed FINAL~n",
                [AnchorId, Seq]
            ),
            error
    end.