%   Exercicse 1                            

close all;
clc;

% criar um robô bem simples com os teus parâmetros
l1 = 10; l2 = 10; l3 = 10; l4 = 10; l6 = 10;

theta_off = deg2rad([  0  -90  -90   0   0   0 ]);
alpha     = deg2rad([ -90  180   90  90 -90   0 ]);
d         = [l1   0    0  l4  0  l6];
a         = [  0   l2   l3   0   0   0 ];

for i = 1:6
    L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', theta_off(i));
end

R = SerialLink(L, 'name', 'Lite6');

q = [0 0 0 0 0 0];

figure; clf;
set(gcf, 'Renderer', 'opengl');   % força renderer decente
R.plot(q, 'workspace', [-50 50 -50 50 0 80], 'noname', 'nowrist');
title('UFACTORY Lite6 — postura home');

title('UFACTORY Lite6 — postura home');

R.teach(q);

%                  R.FKINE                     definir q para algo numerico


% cinemática direta do efetuador
T06_num_SE3 = R.fkine(q);

pos = T06_num_SE3.t; 
Rot = T06_num_SE3.R; 

nx = Rot(1,1); ny = Rot(2,1); nz = Rot(3,1);   % 1ª coluna = n
sx = Rot(1,2); sy = Rot(2,2); sz = Rot(3,2);   % 2ª coluna = s
ax = Rot(1,3); ay = Rot(2,3); az = Rot(3,3);   % 3ª coluna = a

roll  = atan2(ny, nx);
pitch = atan2(-nz, nx*cos(roll) + ny*sin(roll));
yaw   = atan2(sz, az);

rpy_deg = rad2deg([roll, pitch, yaw]);

disp('T_0^6 ='); disp(T06_num_SE3);
disp('Posição [m] ='); disp(pos');
disp('RPY [deg] ='); disp(rpy_deg);