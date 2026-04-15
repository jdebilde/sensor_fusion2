-module(uwb_tag).

-export([
    start/0,
    stop/0,
    ensure_started/0,
    measure_distance/1,
    measure_distance/2,
    loop_test/2
]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(RETRY_DELAY, 200).
-define(UUS_TO_DWT_TIME, 65536).
-define(FINAL_DELAY_UUS, 40000).

-define(TX_ANTD, 16453).
-define(RX_ANTD, 16453).

-define(TS_MASK, 16#FFFFFFFFFF).
-define(TS_WRAP, 16#10000000000).
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

start() ->
    ensure_started().

stop() ->
    case whereis(pmod_uwb) of
        undefined ->
            ok;
        Pid ->
            exit(Pid, kill),
            ok
    end.

ensure_started() ->
    case whereis(pmod_uwb) of
        undefined ->
            case pmod_uwb:start_link(spi2, []) of
                {ok, _Pid} ->
                    configure_uwb(),
                    io:format("UWB tag started~n"),
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

%%% =========================
%%% PUBLIC API
%%% =========================

measure_distance(AnchorId) ->
    measure_distance(AnchorId, 0).

measure_distance(AnchorId, Seq)
  when is_integer(AnchorId), AnchorId >= 0, AnchorId =< 255,
       is_integer(Seq), Seq >= 0, Seq =< 255 ->
    case ensure_started() of
        ok ->
            do_ranging(AnchorId, Seq);
        Error ->
            {error, {start_failed, Error}, Seq}
    end.

loop_test(AnchorId, Seq0) ->
    case measure_distance(AnchorId, Seq0) of
        %% plus tard, quand on aura REPORT:
        {ok, DistanceCm, NextSeq} when is_float(DistanceCm) ->
            io:format(
                "Tag -> anchor ~p seq=~p distance=~.2f cm~n",
                [AnchorId, Seq0, DistanceCm]
            ),
            timer:sleep(?RETRY_DELAY),
            loop_test(AnchorId, NextSeq);

        %% protocole actuel: échange OK mais pas de distance côté tag
        {ok, undefined, NextSeq} ->
            io:format(
                "Tag -> anchor ~p seq=~p exchange ok (distance not reported yet)~n",
                [AnchorId, Seq0]
            ),
            timer:sleep(?RETRY_DELAY),
            loop_test(AnchorId, NextSeq);

        {error, Reason, SameSeq} ->
            io:format(
                "Tag -> anchor ~p seq=~p error: ~p~n",
                [AnchorId, SameSeq, Reason]
            ),
            timer:sleep(?RETRY_DELAY),
            loop_test(AnchorId, SameSeq)
    end.

%%% =========================
%%% INTERNAL DS-TWR
%%% =========================

do_ranging(AnchorId, Seq) ->
    %% ---- T1: POLL ----
    Poll = <<"POLL:", Seq:8, AnchorId:8>>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1_0} = pmod_uwb:read(tx_time),
    T1 = ts_norm(T1_0),

    %% ---- WAIT RESP ----
    case pmod_uwb:reception() of
        {_, <<"RESP:", Seq:8, AnchorId:8, T2_0:40>>} ->
            T2 = ts_norm(T2_0),

            %% ---- T4: receive RESP ----
            #{rx_stamp := T4_0} = pmod_uwb:read(rx_time),
            T4 = ts_norm(T4_0),

            %% ---- T5: delayed FINAL ----
            FinalTxTimeRaw =
                align_delayed_tx_time(
                    T4 + (?FINAL_DELAY_UUS * ?UUS_TO_DWT_TIME)
                ),

            T5 = ts_norm(FinalTxTimeRaw + ?TX_ANTD),

            Final = <<"FINAL:", Seq:8, AnchorId:8, T1:40, T4:40, T5:40>>,

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
                "Tag seq=~p anchor=~p T1=~p T2=~p T4=~p "
                "T5(msg)=~p T5(real)=~p diff=~p~n",
                [Seq, AnchorId, T1, T2, T4, T5, T5Real, Diff]
            ),

            %% Pour l'instant: pas encore de REPORT, donc pas de distance ici
            {ok, undefined, (Seq + 1) band 16#FF};

        {_, <<"RESP:", Seq:8, OtherAnchorId:8, _/binary>>} ->
            {error, {wrong_anchor, OtherAnchorId}, Seq};

        {error, Reason} ->
            {error, {resp_error, Reason}, Seq};

        _ ->
            {error, resp_timeout_or_unexpected, Seq}
    end.