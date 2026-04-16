%% Labwork 3 - Exercise 5 (versão corrigida)
clear; clc; close all;

lg_cm = 10;
lg_m  = lg_cm/100;   % 0.10 m

% ---------- definição dos elos ----------
clear L6 R6
L6 = repmat(Link, 1, 6);

L6(1) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90),  'offset', 0);
L6(2) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90),  'offset', deg2rad(90));
L6(3) = Link('prismatic', 'theta',0,'a', 0, 'alpha', deg2rad(-90));
L6(3).qlim = [0 0.60];
L6(4) = Link('revolute', 'd', 0,    'a', 0, 'alpha',  deg2rad(90));
L6(5) = Link('revolute', 'd', 0,    'a', 0, 'alpha', deg2rad(0));
L6(6) = Link('revolute', 'd', lg_m, 'a', 0, 'alpha',  0,'offset', 0 );
R6 = SerialLink(L6, 'name', 'RRP-RRR 6DOF');

% --- Teste do punho esférico / eixo Z do gripper (opcional) ---
q_test = [0 0 0.30 0 0 0];
T = R6.fkine(q_test);
p = transl(T);        % 3x1
R = t2r(T);           % 3x3

% desenhar robot + eixo +Z do efector para debug
figure('Name','Verificação do punho e eixo Z'); 
R6.plot(q_test);
hold on;
ox = p(1); oy = p(2); oz = p(3);
z_dir = R * [0;0;1];   % direção +Z no sistema global

% esfera na origem do efector
[sx, sy, sz] = sphere(12);
r_sph = 0.01;  % 1 cm
surf(ox + r_sph*sx, oy + r_sph*sy, oz + r_sph*sz, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.6);

% seta do eixo +Z (10 cm)
quiver3(ox, oy, oz, 0.1*z_dir(1), 0.1*z_dir(2), 0.1*z_dir(3), 'LineWidth', 2, 'MaxHeadSize', 1, 'AutoScale','off');
text(ox, oy, oz, '  gripper origin', 'Color','w');

view(3); axis equal; grid on;
hold off;

%% trajectória desejada (círculo no plano z = z_plane)
z_plane = 0.20;
Cx = 0.40;
Cy = 0.00;
r  = 0.15;

% velocidade angular para completar um círculo em Tf2 (ver depois)
alpha_dot = pi/2;   % rad/s -> com Tf2=4 tem 2*pi total
dt   = 0.01;
Tf   = 4;          % círculo completo (usado mais tarde)
t    = 0:dt:Tf;
N    = numel(t);

% ponto inicial desejado (α = 0)
T0_des    = transl(Cx + r, Cy, z_plane);   % 4x4 hom transform
q0_guess  = [0 0 0.30 0 0 0];               % linha
mask      = [1 1 1 0 0 0];                  % só posição

% usar ikine com protecção caso não converja
try
    q0_ik = R6.ikine(T0_des, q0_guess, mask);
catch
    q0_ik = [];
end

if isempty(q0_ik)
    warning('ikine não convergiu, uso q0_guess como configuração inicial');
    q0_ik = q0_guess(:);
else
    % garantir coluna
    q0_ik = q0_ik(:);
end

%% ex5_a  (malha aberta) 
tic;

% Fase 1 – aproximação em z
Tf1 = 2;                 % tempo para ir até ao plano z=0.20
t1  = 0:dt:Tf1;
N1  = numel(t1);

q_phase1 = zeros(6, N1);
q_phase1(:,1) = q0_ik;   % coluna

% ganho P para subida em z
Kz = 5;                  % aumentei para resposta mais rápida
lambda = 1e-3;           % DLS regularization
qdot_max = 0.5;          % m/s or rad/s (limite para qdot) -> ajusta se necessário

for k = 1:N1-1
    T_curr = R6.fkine(q_phase1(:,k)');    % fkine espera linha
    p_curr = transl(T_curr);              % pode ser 1x3 ou 3x1
    if isrow(p_curr), p_curr = p_curr.'; end

    % manter x,y atuais e ir em direção a z_plane
    vz_star = Kz*(z_plane - p_curr(3));   % ganho Kz
    v_star  = [0; 0; vz_star];

    J6 = R6.jacob0(q_phase1(:,k)');     % jacobiana na base
    Jv = J6(1:3,:);                     % parte linear 3x6

    % Damped Least Squares para obter qdot (6x1)
    qdot = (Jv' * Jv + lambda*eye(size(Jv,2))) \ (Jv' * v_star);

    % limitar qdot (component-wise)
    qdot = max(min(qdot, qdot_max), -qdot_max);

    q_phase1(:,k+1) = q_phase1(:,k) + dt*qdot;

    % saturar junta prismática (joint 3)
    if q_phase1(3,k+1) < L6(3).qlim(1)
        q_phase1(3,k+1) = L6(3).qlim(1);
    elseif q_phase1(3,k+1) > L6(3).qlim(2)
        q_phase1(3,k+1) = L6(3).qlim(2);
    end
end

% Fase 2 – o círculo
Tf2 = 4;      % tempo para o círculo completo
t2  = 0:dt:Tf2;
N2  = numel(t2);

q_open    = zeros(6, N2);
qdot_open = zeros(6, N2);

q_open(:,1) = q_phase1(:,end);  % começa onde acabou a fase 1

for k = 1:N2-1
    alpha_k = alpha_dot * t2(k);

    vx_star = -r*sin(alpha_k) * alpha_dot;
    vy_star =  r*cos(alpha_k) * alpha_dot;
    v_star  = [vx_star; vy_star; 0];

    qk  = q_open(:,k)';                 % linha para jacob0/fkine
    J6  = R6.jacob0(qk);
    Jv  = J6(1:3,:);

    % DLS
    qdot = (Jv' * Jv + lambda*eye(size(Jv,2))) \ (Jv' * v_star);

    % limitar qdot para estabilidade/numerics
    qdot = max(min(qdot, qdot_max), -qdot_max);

    qdot_open(:,k) = qdot;
    q_next = q_open(:,k) + dt*qdot;

    % saturar a junta prismática
    if q_next(3) < L6(3).qlim(1)
        q_next(3) = L6(3).qlim(1);
    elseif q_next(3) > L6(3).qlim(2)
        q_next(3) = L6(3).qlim(2);
    end

    q_open(:,k+1) = q_next;
end

% juntar as duas fases para animar:
q_RTB_5a = [q_phase1, q_open].';   % N_total x 6 (cada linha = uma configuração)
N_total = size(q_RTB_5a,1);

dt = 0.01;                 % já definido antes
t_total = 0:dt:dt*(N_total-1);   % tempo correspondente a cada amostra em q_RTB_5a
N_total = numel(t_total);        % reafirmar

% animação: subsample para acelerar (mostrar apenas 1 em cada 'skip' frames)
skip = 3;   % usa 1 para tudo, maior -> mais rápido
q_anim = q_RTB_5a(1:skip:end, :);

figure('Name','ex5_a: aproximação + círculo (anim)');
R6.plot(q_anim, 'tilesize', 0.05);   % 'tilesize' opcional dependendo da versão
title('ex5\_a: aproximação a z=0.20 + círculo');

% Verificação - extrair posições do efector para plot 3D (usa N_total)
P = zeros(3, N_total);
for k = 1:N_total
    T = R6.fkine(q_RTB_5a(k,:));    % fkine aceita linha
    p = transl(T);
    if isrow(p), p = p.'; end
    P(:,k) = p;
end

%% ex5_b  (closed-loop) 
Kp = 20;
lambda = 1e-4;   % regularização para DLS
qdot_max = 1.0;   % rad/s ou m/s, ajusta conforme junta

% usar N_total e t_total definidos antes
q_cl    = zeros(6, N_total);
qdot_cl = zeros(6, N_total);
q_cl(:,1) = q0_ik;   % configuração inicial (coluna)

for k = 1:N_total-1
    alpha_k = alpha_dot * t_total(k);

    % trajectória desejada (posição)
    x_star = Cx + r*cos(alpha_k);
    y_star = Cy + r*sin(alpha_k);
    z_star = z_plane;
    p_star = [x_star; y_star; z_star];   % garantir coluna 3x1

    % estado actual
    qk = q_cl(:,k)';              % linha para fkine/jacob0
    Tq = R6.fkine(qk);

    % garantir que transl devolve coluna 3x1
    p_q = transl(Tq);
    if isrow(p_q)
        p_q = p_q.';
    end

    e = p_star - p_q;    % 3x1

    J6 = R6.jacob0(qk);  % 6x6
    Jv = J6(1:3,:);      % 3x6

    % DLS
    qdot = (Jv' * Jv + lambda*eye(size(Jv,2))) \ (Jv' * (Kp * e));

    % opcional: limitar qdot
    qdot_max = 0.5;
    qdot = max(min(qdot, qdot_max), -qdot_max);

    qdot_cl(:,k) = qdot;
    q_cl(:,k+1)  = q_cl(:,k) + dt*qdot;

    % saturar junta prismática
    if q_cl(3,k+1) < L6(3).qlim(1)
        q_cl(3,k+1) = L6(3).qlim(1);
    elseif q_cl(3,k+1) > L6(3).qlim(2)
        q_cl(3,k+1) = L6(3).qlim(2);
    end
end

q_RTB_5b = q_cl.';   % agora tem N_total linhas

% subsample para animação (muito eficaz)
skip_anim = 5;                 % mostra 1 em cada 5 quadros (aumenta p/ 10 se quiseres mais rápido)
q_anim_a = q_RTB_5a(1:skip_anim:end, :);
q_anim_b = q_RTB_5b(1:skip_anim:end, :);

% animar com fallback se a tua versão suportar 'delay'
figure;
try
    R6.plot(q_anim_b, 'delay', 0.02);   % tenta usar delay
catch
    R6.plot(q_anim_b);                  % fallback
end
title('ex5\_b (subsampled animation)');



%% trajectórias do efetuador no plano XY (robusto)
% assume q_RTB_5a e q_RTB_5b têm tamanho N_total
pG_a = zeros(N_total,3);
pG_b = zeros(N_total,3);
for k = 1:N_total
    T1 = R6.fkine(q_RTB_5a(k,:));
    T2 = R6.fkine(q_RTB_5b(k,:));
    p1 = transl(T1); if isrow(p1), p1 = p1.'; end
    p2 = transl(T2); if isrow(p2), p2 = p2.'; end
    pG_a(k,:) = p1.';   % guardar como linha [x y z]
    pG_b(k,:) = p2.';   % guardar como linha [x y z]
end

% círculo desejado amostrado com N_total pontos
alpha_vec = alpha_dot * t_total;   % t_total tem N_total elementos
xC = Cx + r*cos(alpha_vec);
yC = Cy + r*sin(alpha_vec);

figure('Name','Trajetórias no plano XY');
plot(pG_a(:,1), pG_a(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(pG_b(:,1), pG_b(:,2), 'g--', 'LineWidth', 1.5);
plot(xC,        yC,        'r:', 'LineWidth', 2);
axis equal;
xlabel('X_0 [m]'); ylabel('Y_0 [m]');
legend('ex5\_a','ex5\_b','círculo desejado','Location','best');
grid on;


%% velocidades das juntas em ex5_b
figure;
plot(t_total, qdot_cl.');              % usa t_total, não t
xlabel('t [s]');
ylabel('q̇_i');
legend('q1','q2','d3','q4','q5','q6');
title('ex5\_b – velocidades das juntas');
grid on;

