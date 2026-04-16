%% Labwork 3 - Exercise 3 & 4
clear; clc; close all;

%% ================== 1) Definir robô RRP-R 5 DOF no RTB ==================
lg_cm = 10;             % lg em cm (deve bater com o da IK)
lg_m  = lg_cm/100;      % lg em metros para o RTB

clear L R
L = repmat(Link, 1, 5);

L(1) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90), 'offset', 0);
L(2) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90), 'offset', deg2rad(90));
L(3) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', deg2rad(-90));
L(3).qlim = [0 0.60];
L(4) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90), 'offset', 0);
L(5) = Link('revolute',  'd', lg_m, 'a', 0, 'alpha', 0, 'offset', deg2rad(-90));

R = SerialLink(L, 'name', 'RRP-R 5 DOF');

%% ================== 2) Parâmetros da tarefa e tempo ==================
Cx = 40;               % cm
Cz = 20;               % cm
r  = 30;               % cm
lg = lg_cm;            % cm, tem de bater com a IK

alpha_dot = pi/2;      % rad/s (|ω| do enunciado)
dt   = 0.01;
Tf   = 2;
t    = 0:dt:Tf;
N    = numel(t);

%% ================== 3) EXERCÍCIO 3 — IK analítica ==================
% Trajetória circular usando as equações analíticas da IK do prof.

q    = zeros(4, N);    % [q1; q2; d3; q4] em (rad,rad,cm,rad)
qdot = zeros(4, N);    % [q1dot; q2dot; d3dot; q4dot]

for k = 1:N
    alpha = alpha_dot * t(k);

    % 1) IK analítica (tua função IK_ex1)
    q(:,k) = IK_ex1(alpha, Cx, Cz, r, lg);

    q1 = q(1,k);
    q2 = q(2,k);
    d3 = q(3,k);
    q4 = q(4,k);

    % 2) Velocidades analíticas (deduzidas das equações da IK)
    kgeom = r - lg;    % raio "efetivo" do punho

    q1dot = 0;
    q2dot = -(kgeom/d3) * cos(q4) * alpha_dot;
    d3dot =  (kgeom)    * sin(q4) * alpha_dot;
    q4dot = -alpha_dot - q2dot;

    qdot(:,k) = [q1dot; q2dot; d3dot; q4dot];
end

disp('Primeiras soluções de juntas (q1,q2,d3,q4) — Ex1:');
disp(q(:,1:5));
disp('Primeiras velocidades (q1dot,q2dot,d3dot,q4dot) — Ex1:');
disp(qdot(:,1:5));

% Converter para RTB (em metros, 5 juntas)
q_RTB = zeros(N, 5);
for k = 1:N
    q1 = q(1,k);
    q2 = q(2,k);
    d3 = q(3,k)/100;    % cm -> m
    q4 = q(4,k);
    q5 = 0;             % junta virtual fixa

    q_RTB(k,:) = [q1, q2, d3, q4, q5];
end

fprintf('R.n = %d\n', R.n);
disp('size(q_RTB) = '); disp(size(q_RTB));

% Animar o robô com a solução de IK
figure;
R.plot(q_RTB);
title('Ex1 - Trajetória circular com IK analítica');


%% ================== 4) EXERCÍCIO 4(a) — Jacobiano (no referencial base) ==================
% Movimento usando:
%   q̇*(k) = J(q(k))^{-1} v*(k)
%   q*(k+1)  = q(k) + dt * q̇*(k)
%
% Usamos só as juntas ativas q_a = [q2; d3; q4] (q1 = 0).

q_a     = zeros(3, N);   % [q2; d3; q4]
q_a_dot = zeros(3, N);

% 1) condição inicial: IK para alpha = 0
alpha0 = 0;
q0 = IK_ex1(alpha0, Cx, Cz, r, lg);   % [q1;q2;d3;q4]
q_a(:,1) = q0(2:4);                   % [q2; d3; q4]

for k = 1:N-1

    % ângulo atual na circunferência
    alpha_k = alpha0 + alpha_dot * t(k);

    % 2) velocidade desejada v* = [ẋG; żG; ψ̇] em {0}
    %    xG = Cx + r cos(alpha)
    %    zG = Cz - r sin(alpha)
    vx_star      = -r * sin(alpha_k) * alpha_dot;
    vz_star      = -r * cos(alpha_k) * alpha_dot;
    psi_dot_star = alpha_dot;      % ψ = α => ψ̇ = α̇

    v_star = [vx_star; vz_star; psi_dot_star];

    % 3) Jacobiano 3x3 no ponto atual (no referencial base)
    Jk = J_ex1(q_a(:,k), lg);      % lg em cm, d3 em cm

    % 4) velocidades de junta: q̇* = J^{-1} v*
    qdot_star = Jk \ v_star;
    q_a_dot(:,k) = qdot_star;

    % 5) integração discreta: q(k+1) = q(k) + dt*q̇
    q_a(:,k+1) = q_a(:,k) + dt * qdot_star;
end

% ---- Trajetória completa [q1 q2 d3 q4 q5] para o RTB ----
q_RTB_J = zeros(N, 5);
for k = 1:N
    q1 = 0;
    q2 = q_a(1,k);
    d3 = q_a(2,k)/100;    % cm -> m
    q4 = q_a(3,k);
    q5 = 0;

    q_RTB_J(k,:) = [q1, q2, d3, q4, q5];
end

% ---- Animar o robô com o método do Jacobiano ----
figure;
R.plot(q_RTB_J);
title('Ex4(a) - Movimento com integração numérica (Jacobiano coerente com IK)');



%% ================== 5) EXERCÍCIO 4(b) — Closed-loop com erro de pose ==================
% Agora usamos:
%   q̇*(k) = J(q(k))^{-1} ( p*(k) - f(q(k)) )
%   q(k+1) = q(k) + Kp * dt * q̇*(k)
%
% q_a = [q2; d3; q4], q1 = 0 como antes.

Kp = 5; 
q_cl     = zeros(3, N);   % [q2; d3; q4] - closed-loop
q_cl_dot = zeros(3, N);

% 1) 
alpha0 = 0;
q0 = IK_ex1(alpha0, Cx, Cz, r, lg);   % [q1;q2;d3;q4]
q_cl(:,1) = q0(2:4);                  % [q2; d3; q4]

for k = 1:N-1

    % ângulo atual na circunferência (trajetória desejada)
    alpha_k = alpha0 + alpha_dot * t(k);

    % --------- POSE DESEJADA p*(k) ----------
    x_star   = Cx + r*cos(alpha_k);   % xG* (cm)
    z_star   = Cz - r*sin(alpha_k);   % zG* (cm)
    psi_star = alpha_k;               % ψ* = α (como na IK)

    p_star = [x_star; z_star; psi_star];

    % --------- POSE ATUAL f(q(k)) ----------
    q2 = q_cl(1,k);
    d3 = q_cl(2,k);
    q4 = q_cl(3,k);

    xG = d3*cos(q2) + lg*cos(q2+q4);
    zG = d3*sin(q2) + lg*sin(q2+q4);
    psi = -(q2 + q4);

    f_q = [xG; zG; psi];

    % --------- ERRO DE POSE ----------
    e = p_star - f_q;      % 3x1

    % --------- Jacobiano e q̇*(k) ----------
    Jk = J_ex1(q_cl(:,k), lg);    % mesmo J do 4(a)
    qdot_star = Jk \ e;          

    q_cl_dot(:,k) = qdot_star;

    % --------- Atualização das juntas (closed-loop) ----------
    q_cl(:,k+1) = q_cl(:,k) + Kp * dt * qdot_star;
end

% ---- Construir trajetória completa [q1 q2 d3 q4 q5] para o RTB ----
q_RTB_CL = zeros(N, 5);
for k = 1:N
    q1 = 0;
    q2 = q_cl(1,k);
    d3 = q_cl(2,k)/100;    % cm -> m
    q4 = q_cl(3,k);
    q5 = 0;

    q_RTB_CL(k,:) = [q1, q2, d3, q4, q5];
end

% ---- Animar o robô com o controlo em malha fechada ----
figure;
R.plot(q_RTB_CL);
title(sprintf('Ex4(b) - Closed-loop (Kp = %.1f)', Kp));