-module(uwb_anchor).

-export([start/1, stop/0, ensure_started/0, loop/1, print_delay/0]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TX_ANTD, 16415).
-define(RX_ANTD, 16415).

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

start(AnchorId) when is_integer(AnchorId), AnchorId >= 0, AnchorId =< 255 ->
    case ensure_started() of
        ok ->
            io:format("UWB anchor ~p started~n", [AnchorId]),
            loop(AnchorId);
        Error ->
            Error
    end.

%%% =========================
%%% MAIN LOOP
%%% =========================

loop(AnchorId) ->
    case pmod_uwb:reception() of
        %% POLL targeted to this anchor
        {_, <<"POLL:", Seq:8, AnchorId:8>>} ->
            handle_poll(AnchorId, Seq);

        _ ->
            ok

        % %% POLL for another anchor
        % {_, <<"POLL:", Seq:8, OtherAnchorId:8>>} ->
        %     io:format(
        %         "Anchor ~p ignoring POLL seq=~p for anchor ~p~n",
        %         [AnchorId, Seq, OtherAnchorId]
        %     ),
        %     ok;

        % {error, rxrfto} ->
        %     ok;

        % {error, Reason} ->
        %     io:format("Anchor ~p reception error: ~p~n", [AnchorId, Reason]),
        %     ok;

        % {_, Other} ->
        %     io:format("Anchor ~p unexpected frame: ~p~n", [AnchorId, Other]),
        %     ok;

        % _ ->
        %     ok
    end,
    loop(AnchorId).

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

                    %% ---- REPORT ----
                    %% On encode la distance en entier centi-cm pour éviter les flottants binaires.
                    %% Ex: 93.41 cm -> 9341
                    DistanceCentiCm = round(DistanceCm * 100.0),
                    Report = <<"REPORT:", Seq:8, AnchorId:8, DistanceCentiCm:32/signed>>,

                    pmod_uwb:transmit(Report),

                    % io:format(
                    %     "Anchor ~p seq=~p distance=~.2f cm~n",
                    %     [AnchorId, Seq, DistanceCm]
                    % ),
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

print_delay() ->
    io:format("TX_ANTD=~p RX_ANTD=~p ~n", [?TX_ANTD, ?RX_ANTD]).