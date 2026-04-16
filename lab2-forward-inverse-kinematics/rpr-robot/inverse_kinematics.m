% Alínea c)
clear; clc;

syms q1 d2 q3 q4 real
a1  = sym(2);   % cm
d4  = sym(2);   % cm
pi2 = sym(pi)/2;

q         = [q1, d2, q3, q4];
theta_off = [0 0 0 pi2];
alpha     = [pi2 -pi2 pi2 0];
a         = [a1 0 0 0];
d         = [0 d2 0 d4];
sigma     = [0 1 0 0]; % 0 = R, 1 = P

M_DH = sym(zeros(4,4));

for i = 1:4
    if sigma(i) == 1
        % PRISMÁTICA -> d é variável, theta é fixo (= offset)
        M_DH(i,:) = [alpha(i), a(i), q(i), theta_off(i)];
    else
        % REVOLUTA -> theta é variável, d é fixo
        M_DH(i,:) = [alpha(i), a(i), d(i), q(i) + theta_off(i)];
    end
end

% Matrizes DH individuais
A1 = dh(q1, 0,  a1,  pi2);
A2 = dh(0,  d2, 0,  -pi2);
A3 = dh(q3, 0,  0,  pi2);

% A4 (virtual fixa):
A4 = dh(pi2, d4, 0, 0);

T01 = A1;
T12 = A2;
T2G = A3 * A4;          % 2 -> G (gripper)
T0G = simplify(T01*T12*T2G);

disp("A expressão da Matriz T0G é:")
disp(T0G)

%% Cinemática inversa
syms l1 lg real
syms q1 d2 q3 real
syms x_star y_star phi_star real   % end-effector (x*, y*, φ*)

% 1) tirar o offset do efetuador
X = x_star - lg*sin(phi_star);
Y = y_star - lg*cos(phi_star);

% 2) equação em q1
eq_q1 = X == l1*(cos(q1) + sin(q1)) + Y*tan(q1);

% 3) relação de orientação
eq3 = q3 == phi_star - q1; %#ok<NASGU>

% 4) resolver q1 simbolicamente
S = solve(eq_q1, q1, 'Real', true, 'ReturnConditions', true);
q1_sol = S.q1;   % pode vir com parâmetros

% 5) agora que tens q1, defines d2 e q3
d2_expr = Y/cos(q1_sol) + l1;
q3_expr = phi_star - q1_sol;

% 6) mostrar resultados simbólicos
disp('q1 =');
disp(q1_sol);
disp('d2 =');
disp(d2_expr);
disp('q3 =');
disp(q3_expr);

%% ---- parâmetros do robot ----
l1 = 2;   % cm
lg = 2;   % cm

%% ---- pose desejada do efetuador ----
x_star  = 3;        % cm
y_star  = 1;        % cm
phi_star = pi/4;    % rad

X = x_star - lg*sin(phi_star);
Y = y_star - lg*cos(phi_star);

syms q1
eq_q1 = X == l1*(cos(q1) + sin(q1)) + Y*tan(q1);

% o "0" é o chute inicial
q1_sol = vpasolve(eq_q1, q1, 0);

d2_sol = Y/cos(q1_sol) + l1;
q3_sol = phi_star - q1_sol;

% ---- mostrar resultados numéricos ----
disp('q1 =');
disp(simplify(q1_sol));
disp('d2 =');
disp(simplify(d2_sol));
disp('q3 =');
disp(simplify(q3_sol));

%% --------- DH padrão (θ, d, a, α) ---------
function A = dh(theta, d, a, alpha)
    ct = cos(theta);
    st = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);

    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
          0,   sa,     ca,    d;
          0,   0,      0,     1 ];
end
