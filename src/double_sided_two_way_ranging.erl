-module(double_sided_two_way_ranging).
-export([initiator/0, responder/0]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TIMEOUT, 20000).      % 20 ms
-define(RETRY_DELAY, 200).    % ms
-define(FINAL_DELAY, 1000000). % ~15 ms in DW1000 time units
-define(UUS_TO_DWT_TIME, 65536). % in one UWB µs, there are 65536 t.u (UWB µs are special µs ???)
-define(TX_ANTD, 16400).
-define(RX_ANTD, 16400).

-include_lib("grisp/include/pmod_uwb.hrl").

%%% =========================
%%% INITIATOR
%%% =========================

initiator() ->
    pmod_uwb:start_link(spi2, []),
    % pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    % pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    % pmod_uwb:set_frame_timeout(?TIMEOUT),
    pmod_uwb:set_frame_timeout(16#FFFF),
    io:format("Initiator started~n"),
    loop_initiator(0).

loop_initiator(Seq) ->
    io:format("~n==do_ranging==~n"),
    case do_ranging(Seq) of
        ok ->
            io:format("Ok [~p]...~n", [Seq]),
            timer:sleep(?RETRY_DELAY),
            loop_initiator((Seq + 1) band 16#FF);
        timeout ->
            io:format("Retrying [~p]~n", [Seq]),
            loop_initiator(Seq)
    end.

do_ranging(Seq) ->
    %% ---- T1 ----
    Poll = <<"POLL:", Seq:8>>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),

    %% ---- WAIT RESP ----
    case pmod_uwb:reception() of

        {_, <<"RESP:", Seq:8, T2:40>>} ->
            %% ---- T4 ----
            #{rx_stamp := T4} = pmod_uwb:read(rx_time),

            %% ---- T5 ----
            FinalTXTime = T4 + (40000 * ?UUS_TO_DWT_TIME), % why 30000?
            pmod_uwb:write(dx_time, #{dx_time => FinalTXTime}),
            % T5 = FinalTXTime + ?TX_ANTD,
            T5 = FinalTXTime,
            Final = <<"FINAL:", Seq:8, T1:40, T4:40, T5:40>>,
            pmod_uwb:write(sys_status, #{txfcg => 2#1}),
            % Sending the final message
            pmod_uwb:transmit(Final, #tx_opts{txdlys = ?ENABLED, tx_delay = FinalTXTime}),

            #{tx_stamp := T5_REAL} = pmod_uwb:read(tx_time),
            DIFF = (T5_REAL - T5),
            io:format("Seq=~p T1=~p T4=~p T5=~p (~p)~n", [Seq, T1, T4, T5_REAL, DIFF]),
            ok;

        {error, ANSWER} ->
            io:format("{error, ~p}~n", [ANSWER]),
            ok;

        _ ->
            io:format("_~n"),
            ok
    end.

%%% =========================
%%% RESPONDER
%%% =========================

responder() ->
    pmod_uwb:start_link(spi2, []),
    % pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    % pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    % pmod_uwb:set_frame_timeout(?TIMEOUT),
    pmod_uwb:set_frame_timeout(16#FFFF),
    io:format("Responder started~n"),
    loop_responder().

loop_responder() ->
    io:format("~n==reception()==~n"),
    case pmod_uwb:reception() of

        %% ---- RECEIVE POLL ----
        {_, <<"POLL:", Seq:8>>} ->
            io:format("RECEIVE POLL [~p]...~n", [Seq]),
            handle_poll(Seq);

        {_, ANSWER} ->
            io:format("RECEIVE POLL [~p]...~n", [ANSWER]),
            ok;

        _ ->
            io:format("Missed POLL [_]~n"),
            ok
    end,
    loop_responder().

handle_poll(Seq) ->
    %% ---- T2 ----
    #{rx_stamp := T2} = pmod_uwb:read(rx_time),
    % io:format("T2~n"),

    %% ---- T3 ----
    Resp = <<"RESP:", Seq:8, T2:40>>,
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T3} = pmod_uwb:read(tx_time),
    io:format("Seq=~p T3=~p~n", [Seq, T3]),

    %% ---- T3 ----
    % RespTXTime = T2 + (40000 * ?UUS_TO_DWT_TIME), % why 30000?
    % pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    % % T3 = RespTXTime + ?TX_ANTD,
    % T3 = RespTXTime,
    % Resp = <<"RESP:", Seq:8, T2:40>>,
    % pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    % % Sending the final message
    % pmod_uwb:transmit(Resp, #tx_opts{wait4resp = ?ENABLED, w4r_tim = 20000, txdlys = ?ENABLED, tx_delay = RespTXTime}),
    % #{tx_stamp := T3_REAL} = pmod_uwb:read(tx_time),
    % DIFF = (T3_REAL - T3),
    % io:format("Seq=~p T3_REAL=~p T3=~p (~p)~n", [Seq, T3_REAL, T3, DIFF]),

    case pmod_uwb:reception() of

        {_, <<"FINAL:", Seq:8, T1:40, T4:40, T5:40>>} ->
            %% ---- T6 ----
            #{rx_stamp := T6} = pmod_uwb:read(rx_time),

            %% ---- DS-TWR COMPUTATION ----
            Tround1 = T4 - T1,
            Treply1 = T3 - T2,
            % Treply1 = T3_REAL - T2,
            Tround2 = T6 - T3,
            % Tround2 = T6 - T3_REAL,
            Treply2 = T5 - T4,

            ToF =
                ((Tround1 * Tround2) -
                 (Treply1 * Treply2)) div
                (Tround1 + Tround2 + Treply1 + Treply2),

            Distance =
                ToF * ?DWT_TIME_UNIT * ?C,
            io:format("Seq=~p T1=~p T4=~p T2=~p T3=~p T6=~p T5=~p~n", [Seq, T1, T4, T2, T3, T6, T5]),
            % io:format("Seq=~p T1=~p T4=~p T2=~p T3=~p T6=~p T5=~p~n", [Seq, T1, T4, T2, T3_REAL, T6, T5]),
            io:format("Seq=~p Distance=~p cm~n", [Seq, Distance * 100]),
            io:format("Seq=~p TOF=~p (~p)~n", [Seq, ToF, ToF-213]);

        {error, ANSWER} ->
            io:format("{error, ~p}~n", [ANSWER]),
            ok;

        _ ->
            io:format("_~n"),
            ok
    end.