-module(e_uwb_only_ekf).

-behaviour(hera_measure).

-export([init/1, measure/1]).

-record(st, {
    t0,
    x,
    p,
    seq = 0,
    anchors = [{0.0, 0.0}, {1.0, 0.0}]
}).

init(_Args) ->
    ok = uwb_tag:start(),

    %% Etat = [x, y, vx, vy]
    %% Position initiale manuelle
    X0 = mat:matrix([[1.39], [1.495], [0.0], [0.0]]),
    P0 = mat:diag([0.25, 0.25, 0.5, 0.5]),

    Spec = #{
        name => ?MODULE,
        iter => infinity,
        timeout => 50
    },

    {ok, #st{
        t0 = hera:timestamp(),
        x = X0,
        p = P0
    }, Spec}.

measure(S0 = #st{t0=T0, x=X0, p=P0, seq=Seq0, anchors=[A1,A2]}) ->
    T1 = hera:timestamp(),
    Dt = max(0.01, min(0.20, (T1 - T0) / 1000.0)),

    RangeMsg = uwb_tag:measure_distances([1,2], Seq0),
    Seq1 = next_seq(RangeMsg, Seq0),

    case parse_ranges(RangeMsg) of
        {ok, D1, D2} ->
            case valid_ranges(D1, D2) of
                true ->
                    {X1, P1} = ekf_step(X0, P0, Dt, D1, D2, A1, A2),
                    Values = state_values(X1),

                    {ok, Values, S0#st{
                        t0 = T1,
                        x = X1,
                        p = P1,
                        seq = Seq1
                    }};
                false ->
                    {undefined, S0#st{t0=T1, seq=Seq1}}
            end;
        error ->
            {undefined, S0#st{t0=T1, seq=Seq1}}
    end.

%% ------------------------------------------------------------
%% Parsing UWB
%% ------------------------------------------------------------

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

valid_ranges(D1, D2) ->
    D1 > 0.05 andalso D1 < 10.0 andalso
    D2 > 0.05 andalso D2 < 10.0 andalso
    abs(D1 - D2) < 2.0.

%% ------------------------------------------------------------
%% EKF UWB-only
%% Etat = [x, y, vx, vy]
%% ------------------------------------------------------------

ekf_step(X0, P0, Dt, D1, D2, A1, A2) ->
    Q = mat:diag([0.0025, 0.0025, 0.05, 0.05]),

    %% Bruit UWB : sigma ≈ 30 cm => variance 0.09
    R = mat:diag([0.09, 0.09]),

    Z = mat:matrix([[D1], [D2]]),

    F  = fun(X) -> f(X, Dt) end,
    Jf = fun(_X) -> jf(Dt) end,
    H  = fun(X) -> h(X, A1, A2) end,
    Jh = fun(X) -> jh(X, A1, A2) end,

    {Xa, Pa} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    %% Choisit la branche géométrique la plus proche de l'état précédent
    X1 = choose_closest_branch(Xa, X0, D1, D2),

    %% Limite les vitesses absurdes
    X2 = clamp_velocity(X1, -1.0, 1.0),

    {X2, Pa}.

f(X, Dt) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),
    Vx = mat:get(3, 1, X),
    Vy = mat:get(4, 1, X),

    mat:matrix([
        [Px + Vx * Dt],
        [Py + Vy * Dt],
        [Vx],
        [Vy]
    ]).

jf(Dt) ->
    mat:matrix([
        [1.0, 0.0, Dt,  0.0],
        [0.0, 1.0, 0.0, Dt ],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]).

h(X, {X1,Y1}, {X2,Y2}) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),

    mat:matrix([
        [dist(Px, Py, X1, Y1)],
        [dist(Px, Py, X2, Y2)]
    ]).

jh(X, {X1,Y1}, {X2,Y2}) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),

    D1 = safe_dist(Px, Py, X1, Y1),
    D2 = safe_dist(Px, Py, X2, Y2),

    mat:matrix([
        [(Px-X1)/D1, (Py-Y1)/D1, 0.0, 0.0],
        [(Px-X2)/D2, (Py-Y2)/D2, 0.0, 0.0]
    ]).

%% ------------------------------------------------------------
%% Ambiguïté 2 ancres : choisir le point le plus proche
%% ------------------------------------------------------------

choose_closest_branch(Xa, Xprev, D1, D2) ->
    PrevX = mat:get(1, 1, Xprev),
    PrevY = mat:get(2, 1, Xprev),

    case closest_bilateration_point(D1, D2, PrevX, PrevY) of
        none ->
            Xa;
        {_Bx, By} ->
            X  = mat:get(1, 1, Xa),
            Y  = mat:get(2, 1, Xa),
            Vx = mat:get(3, 1, Xa),
            Vy = mat:get(4, 1, Xa),

            Y1 =
                case By >= 0.0 of
                    true -> abs(Y);
                    false -> -abs(Y)
                end,

            mat:matrix([[X], [Y1], [Vx], [Vy]])
    end.

closest_bilateration_point(D1, D2, PrevX, PrevY) ->
    %% Valable pour ancres {0,0} et {1,0}
    X = (D1*D1 - D2*D2 + 1.0) / 2.0,
    Y2 = D1*D1 - X*X,

    case Y2 < 0.0 of
        true ->
            none;
        false ->
            Yabs = math:sqrt(Y2),
            Ppos = {X, Yabs},
            Pneg = {X, -Yabs},

            case point_dist2(Ppos, {PrevX, PrevY}) =< point_dist2(Pneg, {PrevX, PrevY}) of
                true -> Ppos;
                false -> Pneg
            end
    end.

point_dist2({X1,Y1}, {X2,Y2}) ->
    DX = X1 - X2,
    DY = Y1 - Y2,
    DX*DX + DY*DY.

clamp_velocity(X, MinV, MaxV) ->
    Px = mat:get(1, 1, X),
    Py = mat:get(2, 1, X),
    Vx = clamp(mat:get(3, 1, X), MinV, MaxV),
    Vy = clamp(mat:get(4, 1, X), MinV, MaxV),
    mat:matrix([[Px], [Py], [Vx], [Vy]]).

state_values(X) ->
    [
        mat:get(1, 1, X),
        mat:get(2, 1, X),
        mat:get(3, 1, X),
        mat:get(4, 1, X)
    ].

dist(X,Y,A,B) ->
    math:sqrt((X-A)*(X-A) + (Y-B)*(Y-B)).

safe_dist(X,Y,A,B) ->
    max(1.0e-9, dist(X,Y,A,B)).

clamp(X,A,_B) when X < A -> A;
clamp(X,_A,B) when X > B -> B;
clamp(X,_,_) -> X.