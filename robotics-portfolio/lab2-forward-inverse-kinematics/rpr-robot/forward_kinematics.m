%%   Exercicse 3                           
% ALÍNEA A) B)

syms q1 d2 q3 q4 real
q = [q1 d2 q3 q4];

theta_off = deg2rad([  0  0  0   +90]);
alpha     = deg2rad([ +90  -90   +90  0]);
d         = [0  d2  0  2]; % cm
a         = [2  0  0  0];  % cm

M_DH = sym(zeros(4,4));
sigma = [0 1 0 0];  % 0=R, 1=P
for i = 1:4
    if sigma(i)==1           % PRISMÁTICA -> d é variável, theta é fixo (=offset)
        M_DH(i,:) = [alpha(i), a(i), q(i), theta_off(i)];
    else                     % REVOLUTA   -> theta é variável, d é fixo
        M_DH(i,:) = [alpha(i), a(i), d(i), q(i) + theta_off(i)];
    end
end

clear L R
L = repmat(Link,1,4);   % pré-aloca exatamente 4 links

L(1) = Link('revolute',  'd', 0,     'a', 0.02, 'alpha', deg2rad(90),  'offset', 0);
L(2) = Link('prismatic', 'theta', 0, 'a', 0,    'alpha', deg2rad(-90));
L(3) = Link('revolute',  'd', 0,     'a', 0,    'alpha', deg2rad(90),  'offset', 0);
L(4) = Link('revolute',  'd', 0.02,  'a', 0,    'alpha', 0,            'offset', deg2rad(90));  %Junta virtual
L(2).qlim = [0 0.10]; % 0–10 cm em METROS

R = SerialLink(L,'name','Lite6');

q_home = [0, 0.05, 0, 0];   % [q1 d2 q3 q4], d2 em metros
figure;

R.plot(q_home)
title('Modelo 4-DOF — postura home');
R.teach(q_home);

%%

q1  = deg2rad(10);
d2  = 0.05;           % 5 cm em METROS
q3  = deg2rad(30);
q4  = 0;              % Esta junta é virtual

q   = [q1, d2, q3, q4];   % 1x4

T0G_SE3 = R.fkine(q);   % objeto SE3
T0G     = T0G_SE3.T;    % matriz 4x4
p       = T0G_SE3.t;    % posição 3x1
Rmat    = T0G_SE3.R;    % rotação 3x3
rpy_deg = tr2rpy(T0G, 'deg');

disp('T_0^4 =');       disp(T0G);
disp('Posição [m] ='); disp(p');
disp('RPY [deg] =');   disp(rpy_deg);