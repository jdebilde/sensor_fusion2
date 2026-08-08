-module(sensor_fusion).

-behavior(application).

-export([set_args/1]).
-export([launch/0, launch_all/0, stop_all/0]).
% -export([update_code/2, update_code/3]).
-export([target_nodes/0, connect_nodes/0, connected_target_nodes/0]).
-export([update_code/2, update_code/3]).
-export([check_remote_module/1, check_remote_module/2]).
-export([start/2, stop/1]).
-export([print_debug/0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

set_args(nav2) ->
    C = nav2:calibrate(),
    update_table({{nav2, node()}, C});

set_args(nav3) ->
    Cn = nav3:calibrate(),
    update_table({{nav3, node()}, Cn});

set_args(uwb) ->
    Anchor1 = {1, 2.30, 0.05},
    Anchor2 = {2, 0.10, 0.05},
    Anchors = [Anchor1, Anchor2],
    % Anchor1 = {1, 0, 0},
    % Anchors = [Anchor1],
    Current = 0,
    update_table({{uwb, node()}, { Anchors, Current} }).


launch() ->
    io:format("launch()~n"),
    try launch(node_type()) of
        ok ->
            [grisp_led:color(L, green) || L <- [1, 2]],
            ok
    catch
        error:badarg ->
            [grisp_led:color(L, red) || L <- [1, 2]],
            {error, badarg}
    end.


launch_all() ->
    rpc:multicall(?MODULE, launch, []).


stop_all() ->
    _ = rpc:multicall(application, stop, [hera]),
    _ = rpc:multicall(application, start, [hera]),
    ok.


% %% to be called on the source node
% update_code(Application, Module) ->
%     {ok,_} = c:c(Module),
%     {_,Binary,_} = code:get_object_code(Module),
%     rpc:multicall(nodes(), ?MODULE, update_code,
%         [Application, Module, Binary]).


% %% to be called on the destination node
% update_code(Application, Module, Binary) ->
%     AppFile = atom_to_list(Application) ++ ".app",
%     FullPath = code:where_is_file(AppFile),
%     PathLen = length(FullPath) - length(AppFile),
%     {Path,_} = lists:split(PathLen, FullPath),
%     File = Path ++ atom_to_list(Module) ++ ".beam",
%     ok = file:write_file(File, Binary),
%     c:l(Module).


%% Nodes you expect in the system
target_nodes() ->
    % Hosts = ["nav_1", "nav_2", "nav_3", "uwb_1", "uwb_2", "uwb_3"],
    Hosts = ["nav_3", "uwb_1", "uwb_2", "uwb_3"],
    % Hosts = ["uwb_1", "uwb_3"],
    % Hosts = ["nav_3"],
    [list_to_atom("sensor_fusion@" ++ Host) || Host <- Hosts].


%% Ping all expected nodes
connect_nodes() ->
    [{Node, net_adm:ping(Node)} || Node <- target_nodes()].


%% Return only nodes that are currently connected
connected_target_nodes() ->
    [Node || Node <- target_nodes(), lists:member(Node, nodes())].


%% Compile locally, then update only your target nodes
update_code(Application, Module) ->
    {ok, Module} = c:c(Module),
    {Module, Binary, _Filename} = code:get_object_code(Module),
    %% Make sure all expected nodes are pinged first
    PingResults = connect_nodes(),
    %% Only update nodes that are actually connected
    NodesToUpdate = connected_target_nodes(),
    Results = rpc:multicall(NodesToUpdate, ?MODULE, update_code,
                            [Application, Module, Binary]),
    {PingResults, NodesToUpdate, Results}.


%% Called on the destination nodes
update_code(Application, Module, Binary) ->
    AppFile = atom_to_list(Application) ++ ".app",
    case code:where_is_file(AppFile) of
        non_existing ->
            {error, {app_file_not_found, AppFile}};
        FullPath ->
            Path = filename:dirname(FullPath),
            File = filename:join(Path, atom_to_list(Module) ++ ".beam"),
            ok = file:write_file(File, Binary),
            %% Force reload
            code:purge(Module),
            code:delete(Module),
            code:load_file(Module)
    end.


%% Check one module on one node
check_remote_module(Node, Module) ->
    Beam = rpc:call(Node, code, which, [Module]),

    FileInfo =
        case Beam of
            non_existing ->
                non_existing;
            preloaded ->
                preloaded;
            _ ->
                rpc:call(Node, file, read_file_info, [Beam])
        end,

    Compile = rpc:call(Node, Module, module_info, [compile]),
    Exports = rpc:call(Node, Module, module_info, [exports]),

    {Node, Beam, FileInfo, Compile, Exports}.


%% Check a module on all connected target nodes
check_remote_module(Module) ->
    [{Node, check_remote_module(Node, Module)}
        || Node <- connected_target_nodes()].


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->

    {ok, Supervisor} = sensor_fusion_sup:start_link(),
    io:format("start(_Type, _Args) ~n"),
    init_table(),
    case node_type() of
        nav ->
            [grisp_led:flash(L, red, 500) || L <- [1, 2]],
            _ = grisp:add_device(spi2, pmod_nav),
            % pmod_nav:config(acc, #{odr_g => {hz,14.9}});
            % pmod_nav:config(acc, #{odr_g => {hz,59.5}});
            % pmod_nav:config(acc, #{odr_g => {hz,119}});
            pmod_nav:config(acc, #{odr_g => {hz,238}});
            % pmod_nav:config(acc, #{odr_g => {hz,476}});
            % pmod_nav:config(acc, #{odr_g => {hz,952}});
        uwb ->
            [grisp_led:flash(L, red, 500) || L <- [1, 2]],
            ok;
        _ ->
            _ = net_kernel:set_net_ticktime(8),
            lists:foreach(fun net_kernel:connect_node/1,
                application:get_env(kernel, sync_nodes_optional, []))
    end,
    _ = launch(),
    {ok, Supervisor}.


stop(_State) -> ok.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

node_type() ->
    io:format("node_type()~n"),
    Host = lists:nthtail(14, atom_to_list(node())),
    IsNav = lists:prefix("nav", Host),
    IsUwb = lists:prefix("uwb", Host),
    if
        IsNav -> nav;
        IsUwb -> uwb;
        true -> undefined
    end.

launch(nav) ->
    io:format("launch(nav)~n"),
    io:format("C~n"),
    C = ets:lookup_element(args, {nav2, node()}, 2),
    % io:format("hera:start_measure(nav2)~n"),
    % {ok,_} = hera:start_measure(nav2, C),

    io:format("hera:start_measure(ekf6_nav2_uwb)~n"),
    {ok,_} = hera:start_measure(ekf6_nav2_uwb, C),
    ok;


launch(uwb) ->
    io:format("launch(uwb)~n"),
    io:format("uwb_tag:ensure_started()~n"),
    uwb_tag:ensure_started(),
    io:format("Anchors_and_current~n"),
    Anchors_and_current = ets:lookup_element(args, {uwb, node()}, 2),
    io:format("> ~p ~n", [Anchors_and_current]),
    io:format("hera:start_measure(uwb_measure)~n"),
    {ok,_} = hera:start_measure(uwb_measure, Anchors_and_current),
    % launch(uwb_nav_ekf),
    ok;

launch(uwb_nav_ekf) ->
    NavNode = 'sensor_fusion@nav_3',
    UwbNode = 'sensor_fusion@uwb_3',
    % InitialPos = {0.90, 1.65},
    % InitialPos = {1.50, 1.65},
    % InitialPos = {1.50, 3.15},
    InitialPos = {0.90, 3.15},
    AccBias = {-0.20, 0.46},
    SigmaAcc = 1,
    SigmaUwb = 0.2,
    io:format("uwb_nav_ekf~n"),
    io:format("InitialPos: (~p)~n", [InitialPos]),
    io:format("AccBias   : (~p)~n", [AccBias]),
    io:format("SigmaAcc  : (~p)~n", [SigmaAcc]),
    io:format("SigmaUwb  : (~p)~n", [SigmaUwb]),
    {ok,_} = hera:start_measure(uwb_nav_ekf,
                                {InitialPos, NavNode, UwbNode, AccBias}),
    ok;

launch(_) ->
    io:format("launch(_)~n"),
    ok.


init_table() ->
    io:format("init_table~n"),
    args = ets:new(args, [public, named_table]),
    {ResL,_} = rpc:multicall(nodes(), ets, tab2list, [args]),
    L = lists:filter(fun(Res) ->
        case Res of {badrpc,_} -> false; _ -> true end end, ResL),
    lists:foreach(fun(Object) -> ets:insert(args, Object) end, L).


update_table(Object) ->
    io:format("update_table~n"),
    _ = rpc:multicall(ets, insert, [args, Object]),
    ok.


print_debug() ->
    {ok, Bin} = file:read_file("uwb_nav_ekf_debug.log"),
    io:format("~s", [Bin]).
