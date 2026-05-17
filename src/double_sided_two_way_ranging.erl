-module(double_sided_two_way_ranging).
-export([initiator/0, responder/0]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TIMEOUT, 20000).
-define(RETRY_DELAY, 200).
-define(UUS_TO_DWT_TIME, 65536).
-define(FINAL_DELAY_UUS, 40000).

-define(TX_ANTD, 16453).
-define(RX_ANTD, 16453).

%% DW1000 timestamps are 40-bit
-define(TS_MASK, 16#FFFFFFFFFF).
-define(TS_WRAP, 16#10000000000).

%% Delayed TX must be aligned on 512 ticks
-define(DX_TIME_MASK, 16#FFFFFFFFFFFE00).

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

align_delayed_tx_time(T) ->
    ts_norm(T band ?DX_TIME_MASK).

%%% =========================
%%% INIT / CONFIG
%%% =========================

configure_uwb() ->
    pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    pmod_uwb:set_frame_timeout(16#FFFF).

%%% =========================
%%% INITIATOR
%%% =========================

initiator() ->
    pmod_uwb:start_link(spi2, []),
    configure_uwb(),
    io:format("Initiator started~n"),
    loop_initiator(0).

loop_initiator(Seq) ->
    io:format("~n== do_ranging ==~n"),
    case do_ranging(Seq) of
        ok ->
            io:format("Ok [~p]~n", [Seq]),
            timer:sleep(?RETRY_DELAY),
            loop_initiator((Seq + 1) band 16#FF);
        timeout ->
            io:format("Retrying [~p]~n", [Seq]),
            loop_initiator(Seq);
        error ->
            io:format("Error [~p]~n", [Seq]),
            timer:sleep(?RETRY_DELAY),
            loop_initiator(Seq)
    end.

do_ranging(Seq) ->
    %% ---- T1: POLL ----
    Poll = <<"POLL:", Seq:8>>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1_0} = pmod_uwb:read(tx_time),
    T1 = ts_norm(T1_0),

    %% ---- WAIT RESP ----
    case pmod_uwb:reception() of
        {_, <<"RESP:", Seq:8, T2_0:40>>} ->
            T2 = ts_norm(T2_0),

            %% ---- T4: receive RESP ----
            #{rx_stamp := T4_0} = pmod_uwb:read(rx_time),
            T4 = ts_norm(T4_0),

            %% ---- T5: delayed FINAL ----
            FinalTxTimeRaw =
                align_delayed_tx_time(
                    T4 + (?FINAL_DELAY_UUS * ?UUS_TO_DWT_TIME)
                ),

            %% Timestamp logique à mettre dans FINAL
            %% = delayed tx time + antenna delay
            T5 = ts_norm(FinalTxTimeRaw + ?TX_ANTD),

            Final = <<"FINAL:", Seq:8, T1:40, T4:40, T5:40>>,

            pmod_uwb:write(dx_time, #{dx_time => FinalTxTimeRaw}),
            pmod_uwb:write(sys_status, #{txfcg => 2#1}),
            pmod_uwb:transmit(
                Final,
                #tx_opts{
                    txdlys = ?ENABLED,
                    tx_delay = FinalTxTimeRaw
                }
            ),

            #{tx_stamp := T5Real_0} = pmod_uwb:read(tx_time),
            T5Real = ts_norm(T5Real_0),
            Diff = ts_sub(T5Real, T5),

            io:format(
                "Seq=~p T1=~p T2=~p T4=~p T5(msg)=~p T5(real)=~p diff=~p~n",
                [Seq, T1, T2, T4, T5, T5Real, Diff]
            ),
            ok;

        {error, Reason} ->
            io:format("RESP error: ~p~n", [Reason]),
            timeout;

        _ ->
            io:format("RESP timeout/unexpected~n"),
            timeout
    end.

%%% =========================
%%% RESPONDER
%%% =========================

responder() ->
    pmod_uwb:start_link(spi2, []),
    configure_uwb(),
    io:format("Responder started~n"),
    loop_responder().

loop_responder() ->
    io:format("~n== reception() ==~n"),
    case pmod_uwb:reception() of
        %% ---- RECEIVE POLL ----
        {_, <<"POLL:", Seq:8>>} ->
            io:format("RECEIVE POLL [~p]~n", [Seq]),
            handle_poll(Seq);

        {_, Other} ->
            io:format("Unexpected frame: ~p~n", [Other]),
            ok;

        {error, Reason} ->
            io:format("Reception error: ~p~n", [Reason]),
            ok;

        _ ->
            io:format("Missed POLL~n"),
            ok
    end,
    loop_responder().

handle_poll(Seq) ->
    %% ---- T2: receive POLL ----
    #{rx_stamp := T2_0} = pmod_uwb:read(rx_time),
    T2 = ts_norm(T2_0),

    %% ---- T3: immediate RESP ----
    Resp = <<"RESP:", Seq:8, T2:40>>,
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T3_0} = pmod_uwb:read(tx_time),
    T3 = ts_norm(T3_0),

    io:format("Seq=~p T2=~p T3=~p~n", [Seq, T2, T3]),

    %% ---- WAIT FINAL ----
    case pmod_uwb:reception() of
        {_, <<"FINAL:", Seq:8, T1_0:40, T4_0:40, T5_0:40>>} ->
            T1 = ts_norm(T1_0),
            T4 = ts_norm(T4_0),
            T5 = ts_norm(T5_0),

            %% ---- T6: receive FINAL ----
            #{rx_stamp := T6_0} = pmod_uwb:read(rx_time),
            T6 = ts_norm(T6_0),

            %% ---- DS-TWR with wrap-aware subtraction ----
            Tround1 = ts_sub(T4, T1),
            Treply1 = ts_sub(T3, T2),
            Tround2 = ts_sub(T6, T3),
            Treply2 = ts_sub(T5, T4),

            Den = Tround1 + Tround2 + Treply1 + Treply2,

            case Den of
                0 ->
                    io:format("Seq=~p invalid denominator~n", [Seq]),
                    error;
                _ ->
                    ToF =
                        ((Tround1 * Tround2) - (Treply1 * Treply2)) div Den,

                    DistanceM = ToF * ?DWT_TIME_UNIT * ?C,
                    DistanceCm = DistanceM * 100.0,

                    io:format(
                        "Seq=~p T1=~p T2=~p T3=~p T4=~p T5=~p T6=~p~n",
                        [Seq, T1, T2, T3, T4, T5, T6]
                    ),
                    io:format(
                        "Seq=~p round1=~p reply1=~p round2=~p reply2=~p~n",
                        [Seq, Tround1, Treply1, Tround2, Treply2]
                    ),
                    io:format("Seq=~p ToF=~p~n", [Seq, ToF]),
                    io:format("Seq=~p Distance=~.2f cm~n", [Seq, DistanceCm]),
                    ok
            end;

        {error, Reason} ->
            io:format("FINAL error: ~p~n", [Reason]),
            error;

        _ ->
            io:format("Missed FINAL~n"),
            error
    end.