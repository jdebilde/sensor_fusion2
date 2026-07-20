-module(test_uwb).
-export([inituwb/0]).
-export([rcv/1]).
-export([test1/1, test2/1, test3/1, test4/1, test5/1, test6/1]).
-export([test7/1, test8/1]).

-define(C, 299792458).
-define(DWT_TIME_UNIT, 15.65e-12).

-define(TIMEOUT, 20000).      % 20 ms
-define(RETRY_DELAY, 200).    % ms
-define(FINAL_DELAY, 1000000). % ~15 ms in DW1000 time units
-define(UUS_TO_DWT_TIME, 65536). % in one UWB µs, there are 65536 t.u (UWB µs are special µs ???)
-define(TX_ANTD, 16400).
-define(RX_ANTD, 16400).

-include_lib("grisp/include/pmod_uwb.hrl").

inituwb() ->
    pmod_uwb:start_link(spi2, []),
    pmod_uwb:write(tx_antd, #{tx_antd => ?TX_ANTD}),
    pmod_uwb:write(lde_if, #{lde_rxantd => ?RX_ANTD}),
    % pmod_uwb:set_frame_timeout(16#FFFF),
    io:format("Initiator uwb~n").

rcv(0) ->
    done;

rcv(N) when N > 0 ->
    io:format("~n==reception(~p)==~n", [N]),
    case pmod_uwb:reception() of

        {_, ANSWER} ->
            io:format("RECEIVE [~p]...~n", [ANSWER]),
            ok;

        _ ->
            io:format("ERROR [_]~n"),
            ok
    end,
    rcv(N - 1).

% delay + sys_status + wait4resp
test1(0) ->
    done;

test1(N) when N > 0 ->
    io:format("~n==test1 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{wait4resp = ?ENABLED, w4r_tim = 20000, txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test1(N - 1).

% delay + sys_status + no wait4resp
test2(0) ->
    done;

test2(N) when N > 0 ->
    io:format("~n==test2 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test2(N - 1).

% delay + no sys_status + no wait4resp
test3(0) ->
    done;

test3(N) when N > 0 ->
    io:format("~n==test3 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test3(N - 1).

% delay + no sys_status + no wait4resp
test4(0) ->
    done;

test4(N) when N > 0 ->
    io:format("~n==test4 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    % pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test4(N - 1).

% no delay + no sys_status + no wait4resp
test5(0) ->
    done;

test5(N) when N > 0 ->
    io:format("~n==test5 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    % pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test5(N - 1).

% delay + no sys_status + no wait4resp
test6(0) ->
    done;

test6(N) when N > 0 ->
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    io:format("~n==test6 (~p)==~n", [N]),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    % pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    % pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test6(N - 1).

% no delay + no sys_status + no wait4resp
test7(0) ->
    done;

test7(N) when N > 0 ->
    io:format("~n==test7 (~p)==~n", [N]),
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test7(N - 1).

% delay + no sys_status + no wait4resp
test8(0) ->
    done;

test8(N) when N > 0 ->
    Poll = <<"POLL:">>,
    pmod_uwb:transmit(Poll),
    io:format("~n==test8 (~p)==~n", [N]),
    #{tx_stamp := T1} = pmod_uwb:read(tx_time),
    RespTXTime = T1 + (80000 * ?UUS_TO_DWT_TIME),
    % pmod_uwb:write(dx_time, #{dx_time => RespTXTime}),
    T2 = RespTXTime + ?TX_ANTD,
    Resp = <<"RESP:", T1:40>>,
    pmod_uwb:write(sys_status, #{txfcg => 2#1}),
    pmod_uwb:transmit(Resp, #tx_opts{txdlys = ?ENABLED, tx_delay = RespTXTime}),
    #{tx_stamp := T2_REAL} = pmod_uwb:read(tx_time),
    DIFF = (T2_REAL - T2),
    io:format("T2_REAL=~p T2=~p (~p)~n", [T2_REAL, T2, DIFF]),
    test8(N - 1).
