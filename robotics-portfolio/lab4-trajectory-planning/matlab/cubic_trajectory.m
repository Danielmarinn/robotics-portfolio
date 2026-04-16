%%                  Labwork 4
% NOTA: IK_RPR e J_RPR usam d2 e lg em cm; RTB usa d2 em m.
clear; clc; close all;

%% Definir robô RP-R 5 no RTB 
clear L R
lg_cm = 10; lg_m  = lg_cm/100;      

L = repmat(Link, 1, 3);
L(1) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90), 'offset', deg2rad(90));
L(2) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', deg2rad(-90));
L(2).qlim = [0 0.60];
L(3) = Link('revolute',  'd', 0,   'a', 0, 'alpha', deg2rad(90), 'offset', 0);

R = SerialLink(L, 'name', 'RP-R');
R.tool = trotz(deg2rad(-90)) * transl(0,0,lg_m);

%% Parâmetros da tarefa e tempo
Cx = 40;               % cm
Cy = 20;               % cm
r  = 30;               % cm
lg = lg_cm;            % cm, tem de bater com a IK

% Via-points do enunciado
tk = 0:1:4;                 % 0..4 s
alphak = (0:4)*(pi/2);      % i*pi/2
Tseg = 1;                   % 1 s entre via-points

%% IK nos via-points 
nj = 3;             % junta
nk = numel(tk);     % segmento
qk = zeros(nj, nk);

for i = 1:nk
    qk(:,i) = IK_RPR(alphak(i), Cx, Cy, r, lg); 
end

qk(1,:) = unwrap(qk(1,:));   % theta1 contínuo
qk(3,:) = unwrap(qk(3,:));   % theta3 contínuo 

%% Velocidades nos nós (heurística)
vk = zeros(nj, nk);
vk(:,1)   = 0;   % repouso inicial
vk(:,end) = 0;   % repouso final

for j = 1:nj
    for i = 2:(nk-1)
        Tprev = tk(i)   - tk(i-1);
        Tnext = tk(i+1) - tk(i);

        dq_prev = qk(j,i) - qk(j,i-1);
        dq_next = qk(j,i+1) - qk(j,i);
        

        if dq_prev*dq_next <= 0
            vk(j,i) = 0;
        else
            vk(j,i) = 0.5*(dq_prev/Tprev + dq_next/Tnext);
        end
    end
end


%%  Coeficientes cúbicos por troço
nseg = nk-1;
A = zeros(4, nseg, nj);  % [a0;a1;a2;a3] por (segmento, junta)

for j = 1:nj
    for s = 1:nseg
        Ts = tk(s+1) - tk(s);

        q0 = qk(j,s);   q1 = qk(j,s+1);
        v0 = vk(j,s);   v1 = vk(j,s+1);

        a0 = q0;
        a1 = v0;
        a2 = 3*(q1-q0)/Ts^2 - (2*v0+v1)/Ts;
        a3 = -2*(q1-q0)/Ts^3 + (v0+v1)/Ts^2;

        A(:,s,j) = [a0;a1;a2;a3];
    end
end


%% Amostrar trajetória final e animar
dt = 0.01; Tf = 4;
t = 0:dt:Tf; N = numel(t);

q_RTB_cubic = zeros(N,3);   % [theta1, d2(m), theta3]

for k = 1:N
    if t(k) == tk(end)
        s = nseg;
    else
        s = find(t(k) >= tk(1:end-1) & t(k) < tk(2:end), 1, 'last');
    end
    tau = t(k) - tk(s);

    q_now = zeros(3,1);
    for j = 1:nj
        a = A(:,s,j);
        q_now(j) = a(1) + a(2)*tau + a(3)*tau^2 + a(4)*tau^3;
    end

    theta1 = q_now(1);
    d2_cm = q_now(2);
    d2_m  = d2_cm/100;
    theta3 = q_now(3);
    q_RTB_cubic(k,:) = [theta1, d2_m, theta3];
end

figure; R.plot(q_RTB_cubic); title('RP-R: trajetória circular via-points (cúbicas)');

%%  Labwork 4 — 3(a) Jacobiano com alpha_dot imposto
% NOTA: q = [theta1; d2; theta3] com d2 em cm; J_RPR também usa cm.
alpha0    = 0;
alpha_dot = +pi/2;  % rad/s


% q em unidades do Jacobiano: [theta1; d2(cm); theta3]
q = zeros(3,N);
q(:,1) = IK_RPR(alpha0, Cx, Cy, r, lg);


for k = 1:N-1
    alpha = alpha0 + alpha_dot*t(k);

    % v do gripper em cm/s (e psi_dot em rad/s)
    xdot   = -r*sin(alpha)*alpha_dot;
    ydot   =  r*cos(alpha)*alpha_dot;
    psidot = alpha_dot; % psi = alpha  (orientação do gripper igual ao parâmetro da trajetória)
    v = [xdot; ydot; psidot];
    J = J_RPR(q(:,k), lg);        % d2 e lg em cm
    qdot = J \ v;                

    q(:,k+1) = q(:,k) + dt*qdot;
end

% Converter para RTB (prismática em m)
q_RTB_jacob = [q(1,:).', q(2,:).'/100, q(3,:).'];
figure; R.plot(q_RTB_jacob); title('3(a) - Jacobiano com alpha\_dot imposto');

%% Comparar os dois métodos 

N = min(size(q_RTB_cubic,1), size(q_RTB_jacob,1));
pC = zeros(N,3); pJ = zeros(N,3);

for k = 1:N
    pC(k,:) = transl(R.fkine(q_RTB_cubic(k,:)));
    pJ(k,:) = transl(R.fkine(q_RTB_jacob(k,:)));
end

figure;
plot(pC(:,1), pC(:,2),'y', pJ(:,1), pJ(:,2),'r--');
axis equal; grid on;
legend('Ex1 (cúbicas)','3a (Jacobiano)');
title('Comparação da trajetória do gripper no plano XY');

d = vecnorm(pC(:,1:2) - pJ(:,1:2), 2, 2);
fprintf('Diferença média entre métodos (XY): %.4f m\n', mean(d));
fprintf('Diferença máxima entre métodos (XY): %.4f m\n', max(d));