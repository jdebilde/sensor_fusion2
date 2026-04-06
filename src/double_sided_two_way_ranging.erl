-module(double_sided_two_way_ranging).
-export([initiator/0, responder/0]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TIMEOUT, 20000). % 20 ms
-define(RETRY_DELAY, 200). % ms

%%% =========================
%%% INITIATOR
%%% =========================

initiator() ->
    pmod_uwb:start_link(spi2, []),
    pmod_uwb:set_frame_timeout(?TIMEOUT),
    io:format("Initiator started~n"),
    loop_initiator(0).

loop_initiator(Seq) ->
    io:format("do_ranging...~n"),
    case do_ranging(Seq) of
        ok ->
            io:format("Ok [~p]...~n", [Seq]),
            timer:sleep(?RETRY_DELAY),
            loop_initiator((Seq + 1) band 16#FF);

        timeout ->
            io:format("Retrying [~p]...~n", [Seq]),
            loop_initiator(Seq)
    end.

do_ranging(Seq) ->
    %% ---- STEP 1: SEND POLL ----
    Poll = <<"POLL:", Seq:8>>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),

    %% ---- STEP 2: WAIT RESP ----
    case pmod_uwb:reception() of
        % {_, <<"RESP:", Seq:8, T2:40, T3:40>>} ->
        {_, <<"RESP:", Seq:8, T2:40>>} ->
            #{rx_stamp := T4} = pmod_uwb:read(rx_time),
            #{tx_stamp := T3} = pmod_uwb:read(tx_time),

            Final = <<"FINAL:", Seq:8, T1:40, T4:40>>,
            pmod_uwb:transmit(Final),
            #{tx_stamp := T5} = pmod_uwb:read(tx_time),

            io:format("Seq=~p T1=~p T4=~p~n", [Seq, T1, T4]),
            io:format("Seq=~p T2=~p T3=~p?~n", [Seq, T2, T3]),
            io:format("Seq=~p T5=~p~n", [Seq, T5]),
            io:format("Sent FINAL (~p)~n", [Seq]),
            ok;

        {_, rxrfto} ->
            io:format("Unexpected RESP: [rxrfto]~n"),
            timeout;

        {_, Other} ->
            io:format("Unexpected RESP: ~p~n", [Other]),
            timeout
    end.

%%% =========================
%%% RESPONDER
%%% =========================

responder() ->
    pmod_uwb:start_link(spi2, []),
    pmod_uwb:set_frame_timeout(?TIMEOUT),
    io:format("Responder started~n"),
    loop_responder().

loop_responder() ->
    io:format("reception...~n"),
    case pmod_uwb:reception() of

        %% ---- STEP 1: RECEIVE POLL ----
        {_, <<"POLL:", Seq:8>>} ->
            io:format("RECEIVE POLL [~p]...~n", [Seq]),
            handle_poll(Seq);

        %% Ignore everything else
        {error, _} ->
            io:format("error...~n"),
            ok;

        _ ->
            io:format("_...~n"),
            ok
    end,
    loop_responder().

handle_poll(Seq) ->
    #{rx_stamp := T2} = pmod_uwb:read(rx_time),

    %% Send RESP immediately
    Resp = <<"RESP:", Seq:8, T2:40>>,
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T3} = pmod_uwb:read(tx_time),

    %% Do NOT wait strictly for FINAL
    wait_final(Seq, T2, T3).

wait_final(Seq, T2, T3) ->
    case pmod_uwb:reception() of
        {_, <<"FINAL:", Seq:8, T1:40, T4:40>>} ->
            #{rx_stamp := T6} = pmod_uwb:read(rx_time),
            Tround = T4 - T1,
            Treply = T3 - T2,
            ToF = (Tround - Treply) div 2,
            Distance = ToF * ?DWT_TIME_UNIT * ?C,
            io:format("Seq=~p T1=~p T4=~p~n", [Seq, T1, T4]),
            io:format("Seq=~p T2=~p T3=~p~n", [Seq, T2, T3]),
            io:format("Seq=~p T6=~p~n", [Seq, T6]),
            io:format("Seq=~p Distance=~p m~n", [Seq, Distance]);

        rxrfto ->
            ok;

        {error, _} ->
            ok;

        _ ->
            ok
    end.