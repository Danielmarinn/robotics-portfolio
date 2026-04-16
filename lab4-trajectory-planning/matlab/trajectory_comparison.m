%% Labwork 4 - Aula 2, Ex.4 (parabólicas com limites de aceleração)
clear; clc; close all;

%% Robô RP-R + tool (igual à Aula 1)
lg_cm = 10; lg_m = lg_cm/100;
L = repmat(Link, 1, 3);
L(1) = Link('revolute',  'd', 0, 'a', 0, 'alpha', deg2rad(90),  'offset', deg2rad(90));
L(2) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', deg2rad(-90)); L(2).qlim = [0 0.60];
L(3) = Link('revolute',  'd', 0, 'a', 0, 'alpha', deg2rad(90),  'offset', 0);
R = SerialLink(L, 'name', 'RP-R');
R.tool = trotz(deg2rad(-90)) * transl(0,0,lg_m);

Cx = 40;  Cy = 20;  r = 30;  lg = lg_cm;
tk = 0:1:4;                 % 0..4 s
alphak = (0:4)*(pi/2);      % i*pi/2
Tseg = diff(tk);            % [1 1 1 1]

%% IK nos via-points (juntas: [theta1; d2(cm); theta3])
nj = 3; nk = numel(tk);
qk = zeros(nj, nk);
for i = 1:nk
    qk(:,i) = IK_RPR(alphak(i), Cx, Cy, r, lg);
end
qk(1,:) = unwrap(qk(1,:));
qk(3,:) = unwrap(qk(3,:));

%% Limites de aceleração
a_max_theta = deg2rad(500);  % rad/s^2
a_max_d2    = 150;           % cm/s^2

%% Trajetória parabólica junta a junta
dt = 0.01;
t_total = [];
q_para  = [];
qd_para = [];
qdd_para= [];

for s = 1:(nk-1)
    T = Tseg(s);

    [q1, q1d, q1dd, t_seg] = parabolic_segment(qk(1,s), qk(1,s+1), T, a_max_theta, dt);
    [q2, q2d, q2dd, ~]     = parabolic_segment(qk(2,s), qk(2,s+1), T, a_max_d2,    dt);
    [q3, q3d, q3dd, ~]     = parabolic_segment(qk(3,s), qk(3,s+1), T, a_max_theta, dt);

    if s == 1
        t_total = t_seg;
        q_para  = [q1 q2 q3];
        qd_para = [q1d q2d q3d];
        qdd_para= [q1dd q2dd q3dd];
    else
        % evitar repetir o instante inicial
        t_seg = t_total(end) + t_seg(2:end);
        t_total = [t_total; t_seg];
        q_para  = [q_para;   [q1(2:end)  q2(2:end)  q3(2:end)]];
        qd_para = [qd_para;  [q1d(2:end) q2d(2:end) q3d(2:end)]];
        qdd_para= [qdd_para; [q1dd(2:end) q2dd(2:end) q3dd(2:end)]];
    end
end

%% Verificar limites de aceleração
max_a_theta1 = max(abs(qdd_para(:,1)));
max_a_d2     = max(abs(qdd_para(:,2)));
max_a_theta3 = max(abs(qdd_para(:,3)));

fprintf('max |theta1_dd| = %.3f rad/s^2 (limite = %.3f)\n', max_a_theta1, a_max_theta);
fprintf('max |d2_dd|     = %.3f cm/s^2 (limite = %.3f)\n',   max_a_d2,     a_max_d2);
fprintf('max |theta3_dd| = %.3f rad/s^2 (limite = %.3f)\n', max_a_theta3, a_max_theta);

%% Converter para RTB e animar
q_RTB_para = [ q_para(:,1), q_para(:,2)/100, q_para(:,3) ];
figure; R.plot(q_RTB_para); title('Ex4 - Trajetória com funções parabólicas (limites de aceleração)');

%% Perfis q(t), qd(t), qdd(t) para a junta 1 (theta1)
figure;

subplot(3,1,1);
plot(t_total, q_para(:,1),'LineWidth',1.5);
grid on;
ylabel('\theta_1 (rad)');
title('Perfil de posição, velocidade e aceleração - \theta_1 (parabolic blends)');
legend('\theta_1(t)','Location','best');

subplot(3,1,2);
plot(t_total, qd_para(:,1),'LineWidth',1.5);
grid on;
ylabel('theta1 dot (rad/s)');
legend('d\theta_1/dt','Location','best');

subplot(3,1,3);
plot(t_total, qdd_para(:,1),'LineWidth',1.5);
grid on;
ylabel('theta1 ddot (rad/s^2)');
xlabel('t (s)');
legend('d^2\theta_1/dt^2','Location','best');


