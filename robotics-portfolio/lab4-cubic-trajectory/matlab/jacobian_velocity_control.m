%% Labwork 4 — 3(a) Jacobiano com alpha_dot imposto
clear; clc; close all;

%% Robô RP-R (3 DOF) + tool
lg_cm = 10; lg_m = lg_cm/100;

L = repmat(Link, 1, 3);
L(1) = Link('revolute',  'd', 0, 'a', 0, 'alpha', deg2rad(90),  'offset', deg2rad(90));
L(2) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', deg2rad(-90));
L(2).qlim = [0 0.60];                      % m
L(3) = Link('revolute',  'd', 0, 'a', 0, 'alpha', deg2rad(90),  'offset', 0);

R = SerialLink(L, 'name', 'RP-R');
R.tool = trotz(deg2rad(-90)) * transl(0,0,lg_m);

%% Tarefa (plano XY do enunciado)
Cx = 40;  Cy = 20;  r = 30;  lg = lg_cm;

alpha0    = 0;
alpha_dot = +pi/2;   % ou -pi/2 (rad/s)

dt = 0.01; Tf = 4;
t  = 0:dt:Tf;  N = numel(t);

% q em unidades do Jacobiano: [theta1; d2(cm); theta3]
q = zeros(3,N);
q(:,1) = IK_RPR(alpha0, Cx, Cy, r, lg);

for k = 1:N-1
    alpha = alpha0 + alpha_dot*t(k);

    % v do gripper em cm/s (e psi_dot em rad/s)
    xdot   = -r*sin(alpha)*alpha_dot;
    ydot   =  r*cos(alpha)*alpha_dot;

    % Se no teu modelo psi = -(theta1+theta3), então impor psi=alpha dá psidot=alpha_dot
    psidot = alpha_dot;

    v = [xdot; ydot; psidot];

    J = J_RPR(q(:,k), lg);        % d2 e lg em cm
    qdot = J \ v;                 % resolve sistema linear J*qdot=v 

    q(:,k+1) = q(:,k) + dt*qdot;
end

% Converter para RTB (prismática em m)
q_RTB = [q(1,:).', q(2,:).'/100, q(3,:).'];
figure; R.plot(q_RTB); title('3(a) - Jacobiano com alpha\_dot imposto');
