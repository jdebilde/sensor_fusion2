-module(uwb_range).
-export([initiator/0, responder/0]).

-define(C, 299792458).           % سرعة الضوء (m/s)
-define(DWT_TIME_UNIT, 15.65e-12). % DW1000 time unit (seconds)

%%% =========================
%%% INITIATOR (BOARD A)
%%% =========================
initiator() ->
    pmod_uwb:start_link(spi2, []),
    io:format("Initiator started~n"),
    loop_initiator().

loop_initiator() ->
    timer:sleep(1000),

    %% 1. Send POLL
    pmod_uwb:transmit(<<"POLL">>),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),

    %% 2. Wait for RESPONSE
    {_, Resp} = pmod_uwb:reception(),

    case Resp of
        <<"RESP:", T2:40, T3:40>> ->
            #{rx_stamp := T4} = pmod_uwb:read(rx_time),

            %% Compute ToF
            Tround = T4 - T1,
            Treply = T3 - T2,
            ToF = (Tround - Treply) div 2,

            %% Convert to distance
            Distance =
                ToF * ?DWT_TIME_UNIT * ?C,

            io:format("T1=~p T2=~p T3=~p T4=~p~n", [T1, T2, T3, T4]),
            io:format("ToF=~p  Distance=~p meters~n~n", [ToF, Distance]);

        _ ->
            io:format("Unexpected response: ~p~n", [Resp])
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
    %% 1. Wait for POLL
    {_, Msg} = pmod_uwb:reception(),

    case Msg of
        <<"POLL">> ->
            #{rx_stamp := T2} = pmod_uwb:read(rx_time),

            %% Small delay to simulate processing (optional)
            timer:sleep(5),

            %% 2. Send RESPONSE (include timestamps)
            pmod_uwb:transmit(<< "RESP:", T2:40, 0:40 >>),

            %% IMPORTANT: read TX timestamp AFTER sending
            #{tx_stamp := T3} = pmod_uwb:read(tx_time),

            %% Re-send with correct T3 (better accuracy)
            pmod_uwb:transmit(<< "RESP:", T2:40, T3:40 >>),

            io:format("Handled POLL: T2=~p T3=~p~n", [T2, T3]);

        _ ->
            io:format("Unknown message: ~p~n", [Msg])
    end,

    loop_responder().