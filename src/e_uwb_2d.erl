-module(e_uwb_2d).

-behaviour(hera_measure).

-export([init/1, measure/1]).

-record(st, {
    t0,
    x,
    p,
    seq = 0,
    nav_node = 'sensor_fusion@nav_1',
    anchors = [{0.0, 0.0}, {2.0, 0.0}]
}).

init(_Args) ->
    ok = uwb_tag:start(),

    %% Etat = [x, y, vx, vy, theta]
    X0 = mat:matrix([[1.39], [1.495], [0.0], [0.0], [-math:pi()/2.0]]),
    P0 = mat:diag([0.25, 0.25, 0.5, 0.5, 0.2]),

    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 50
    },

    {ok, #st{t0 = hera:timestamp(), x = X0, p = P0}, Spec}.

measure(S0 = #st{t0=T0, x=X0, p=P0, seq=Seq0,
                  nav_node=NavNode, anchors=[A1,A2]}) ->
    T1 = hera:timestamp(),
    Dt = max(0.01, min(0.20, (T1 - T0) / 1000.0)),

    case latest_nav3(NavNode) of
        {ok, _AxB, _AyB, Gz, Mx, My} ->
            RangeMsg = uwb_tag:measure_distances([1,2], Seq0),
            Seq1 = next_seq(RangeMsg, Seq0),

            case parse_ranges(RangeMsg) of
                {ok, D1, D2} ->
                    {X1, P1} = ekf_step(X0, P0, Dt, Gz, Mx, My, D1, D2, A1, A2),
                    Values = state_values(X1),
                    {ok, Values, S0#st{t0=T1, x=X1, p=P1, seq=Seq1}};
                error ->
                    {undefined, S0#st{t0=T1, seq=Seq1}}
            end;
        error ->
            {undefined, S0#st{t0=T1}}
    end.

latest_nav3(NavNode) ->
    case hera_data:get(nav3, NavNode) of
        [] ->
            error;
        Data ->
            case lists:last(Data) of
                {_, _, _Ts, [_Ax,_Ay,_Az,_Gx,_Gy,Gz,Mx,My,_Mz]} ->
                    {ok, _Ax, _Ay, Gz, Mx, My};
                _ ->
                    error
            end
    end.

parse_ranges({ok, L, _Seq}) ->
    case {lists:keyfind(1, 1, L), lists:keyfind(2, 1, L)} of
        {{1, D1cm}, {2, D2cm}} ->
            {ok, D1cm / 100.0, D2cm / 100.0};
        _ ->
            error
    end;
parse_ranges(_) ->
    error.

next_seq({ok, _L, Seq}, _OldSeq) -> Seq;
next_seq(_, OldSeq) -> OldSeq.

ekf_step(X0, P0, Dt, Gz, Mx, My, D1, D2, A1, A2) ->
    %% UWB est la source principale.
    %% NAV influence faiblement theta via gyro + mag.
    Q = mat:diag([0.001, 0.001, 0.10, 0.10, 0.05]),
    R = mat:diag([0.04, 0.04, 0.50]),

    ThetaMag = heading(Mx, My),
    Z = mat:matrix([[D1], [D2], [ThetaMag]]),

    F  = fun(X) -> f(X, Dt, Gz) end,
    Jf = fun(_X) -> jf(Dt) end,
    H  = fun(X) -> h(X, A1, A2) end,
    Jh = fun(X) -> jh(X, A1, A2) end,

    {Xa, Pa} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    Xb = choose_closest_branch(Xa, X0, D1, D2),
    X1 = clamp_velocity(Xb, -1.0, 1.0),

    {X1, Pa}.

%% Etat = [x,y,vx,vy,theta]
%% Modèle stable : vitesse constante.
%% On n'intègre PAS Ax/Ay du NAV pour éviter la divergence.
f(X, Dt, Gz) ->
    Px = mat:get(1,1,X),
    Py = mat:get(2,1,X),
    Vx = mat:get(3,1,X),
    Vy = mat:get(4,1,X),
    Th = mat:get(5,1,X),

    mat:matrix([
        [Px + Vx * Dt],
        [Py + Vy * Dt],
        [Vx],
        [Vy],
        [wrap(Th + Gz * Dt)]
    ]).

jf(Dt) ->
    mat:matrix([
        [1.0,0.0,Dt, 0.0,0.0],
        [0.0,1.0,0.0,Dt, 0.0],
        [0.0,0.0,1.0,0.0,0.0],
        [0.0,0.0,0.0,1.0,0.0],
        [0.0,0.0,0.0,0.0,1.0]
    ]).

h(X, {X1,Y1}, {X2,Y2}) ->
    Px = mat:get(1,1,X),
    Py = mat:get(2,1,X),
    Th = mat:get(5,1,X),

    mat:matrix([
        [dist(Px,Py,X1,Y1)],
        [dist(Px,Py,X2,Y2)],
        [Th]
    ]).

jh(X, {X1,Y1}, {X2,Y2}) ->
    Px = mat:get(1,1,X),
    Py = mat:get(2,1,X),

    D1 = safe_dist(Px,Py,X1,Y1),
    D2 = safe_dist(Px,Py,X2,Y2),

    mat:matrix([
        [(Px-X1)/D1, (Py-Y1)/D1, 0.0, 0.0, 0.0],
        [(Px-X2)/D2, (Py-Y2)/D2, 0.0, 0.0, 0.0],
        [0.0,        0.0,        0.0, 0.0, 1.0]
    ]).

choose_closest_branch(Xa, Xprev, D1, D2) ->
    PrevX = mat:get(1,1,Xprev),
    PrevY = mat:get(2,1,Xprev),

    case closest_bilateration_point(D1, D2, PrevX, PrevY) of
        none ->
            Xa;
        {_Bx, By} ->
            X  = mat:get(1,1,Xa),
            Y  = mat:get(2,1,Xa),
            Vx = mat:get(3,1,Xa),
            Vy = mat:get(4,1,Xa),
            Th = mat:get(5,1,Xa),

            Y1 =
                case By >= 0.0 of
                    true -> abs(Y);
                    false -> -abs(Y)
                end,

            mat:matrix([[X],[Y1],[Vx],[Vy],[Th]])
    end.

closest_bilateration_point(D1, D2, PrevX, PrevY) ->
    %% Valable pour les ancres {0,0} et {2,0}
    L = 2.0,
    X = (D1*D1 - D2*D2 + L*L) / (2.0 * L),
    Y2 = D1*D1 - X*X,

    case Y2 < 0.0 of
        true ->
            none;
        false ->
            Yabs = math:sqrt(Y2),
            Ppos = {X, Yabs},
            Pneg = {X, -Yabs},

            case point_dist2(Ppos, {PrevX,PrevY}) =< point_dist2(Pneg, {PrevX,PrevY}) of
                true -> Ppos;
                false -> Pneg
            end
    end.

clamp_velocity(X, MinV, MaxV) ->
    Px = mat:get(1,1,X),
    Py = mat:get(2,1,X),
    Vx = clamp(mat:get(3,1,X), MinV, MaxV),
    Vy = clamp(mat:get(4,1,X), MinV, MaxV),
    Th = mat:get(5,1,X),
    mat:matrix([[Px],[Py],[Vx],[Vy],[Th]]).

state_values(X) ->
    [
        mat:get(1,1,X),
        mat:get(2,1,X),
        mat:get(3,1,X),
        mat:get(4,1,X),
        mat:get(5,1,X)
    ].

heading(Mx, My) ->
    math:atan2(My, Mx).

point_dist2({X1,Y1}, {X2,Y2}) ->
    DX = X1-X2,
    DY = Y1-Y2,
    DX*DX + DY*DY.

dist(X,Y,A,B) ->
    math:sqrt((X-A)*(X-A) + (Y-B)*(Y-B)).

safe_dist(X,Y,A,B) ->
    max(1.0e-9, dist(X,Y,A,B)).

clamp(X,A,_B) when X < A -> A;
clamp(X,_A,B) when X > B -> B;
clamp(X,_,_) -> X.

wrap(A) ->
    TwoPi = 2.0 * math:pi(),
    A - TwoPi * math:floor((A + math:pi()) / TwoPi).