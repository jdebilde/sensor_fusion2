-module(double_sided_two_way_ranging).
-export([initiator/0, responder/0]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

%%% =========================
%%% INITIATOR (BOARD A)
%%% =========================

initiator() ->
    pmod_uwb:start_link(spi2, []),
    io:format("Initiator started~n"),
    loop_initiator().

loop_initiator() ->
    timer:sleep(1000),
    io:format("initiator: OK~n"),

    %% ---- STEP 1: POLL ----
    io:format("initiator: transmit~n"),
    pmod_uwb:transmit(<<"POLL">>),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),

    %% ---- STEP 2: WAIT RESP ----
    io:format("initiator: reception~n"),
    case pmod_uwb:reception() of
        {_, <<"RESP:", T2:40, T3:40>>} ->

            #{rx_stamp := T4} = pmod_uwb:read(rx_time),

            %% ---- STEP 3: SEND FINAL ----
            FinalMsg = <<"FINAL:", T1:40, T4:40>>,
            io:format("initiator: transmit 2~n"),
            pmod_uwb:transmit(FinalMsg),
            #{tx_stamp := T5} = pmod_uwb:read(tx_time),

            io:format("Sent FINAL T1=~p T4=~p T5=~p~n", [T1, T4, T5]);

        {error, Reason} ->
            io:format("RX error: ~p~n", [Reason]);

        Other ->
            io:format("Unexpected: ~p~n", [Other])
    end,

    loop_initiator().

%%% =========================
%%% RESPONDER (BOARD B)
%%% =========================

responder() ->
    pmod_uwb:start_link(spi2, []),
    io:format("Responder started~n"),
    loop_responder().

loop_responder() ->
    %% ---- STEP 1: WAIT POLL ----
    io:format("responder: reception~n"),
    case pmod_uwb:reception() of
        {_, <<"POLL">>} ->

            #{rx_stamp := T2} = pmod_uwb:read(rx_time),

            %% ---- STEP 2: SEND RESP ----
            io:format("responder: transmit~n"),
            %% Send placeholder first (we fix below)
            pmod_uwb:transmit(<<0>>),
            #{tx_stamp := T3} = pmod_uwb:read(tx_time),

            %% Now send correct RESP with timestamps
            RespMsg = <<"RESP:", T2:40, T3:40>>,
            io:format("responder: transmit 2~n"),
            pmod_uwb:transmit(RespMsg),

            %% ---- STEP 3: WAIT FINAL ----
            io:format("responder: reception 2~n"),
            case pmod_uwb:reception() of
                {_, <<"FINAL:", T1:40, T4:40>>} ->

                    #{rx_stamp := T6} = pmod_uwb:read(rx_time),

                    %% ---- DS-TWR COMPUTATION ----
                    Tround1 = T4 - T1,
                    Treply1 = T3 - T2,
                    Tround2 = T6 - T3,
                    Treply2 = T5 = T4 - T1, %% approximate (initiator delay small)

                    %% Simplified symmetric formula
                    ToF =
                        ((Tround1 - Treply1) +
                         (Tround2 - Treply2)) div 4,

                    Distance =
                        ToF * ?DWT_TIME_UNIT * ?C,

                    io:format("T2=~p T3=~p T6=~p~n", [T2, T3, T6]),
                    io:format("ToF=~p Distance=~p m~n~n", [ToF, Distance]);

                {error, R2} ->
                    io:format("FINAL RX error: ~p~n", [R2]);

                Other2 ->
                    io:format("Unexpected FINAL: ~p~n", [Other2])
            end;

        {error, Reason} ->
            io:format("RX error: ~p~n", [Reason]);

        _ ->
            ok
    end,

    loop_responder().