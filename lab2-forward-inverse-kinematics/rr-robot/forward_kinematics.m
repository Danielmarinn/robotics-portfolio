    
%   Exercicse 2                            

%   ALÍNEA a) b) e c)
clc; clear; close all;

l1=25;l2=10;lg=2;
theta_off = deg2rad([  0  0  +90   +90]);
alpha     = deg2rad([ -90  +90   +90  0]);
d         = [-l1  +l2  0  +lg];
a         = [+l2  +l1  0   0];



%   Posições i,ii,iii
q = [0 0 0 0];
% q = deg2rad([0 90 90 0]);
% q = deg2rad([10 -45 +30 0]);

PJ_DH = sym(zeros(4,4));
for i = 1:4
    PJ_DH(i,:) = [alpha(i), a(i), d(i), q(i) + theta_off(i)];
end

[A_rel, T04_sym] = MGD_DH(PJ_DH);   

T02 =A_rel{1} * A_rel{2};
fprintf('Matriz de 0 a 2\n');
disp(T02)

T04 =A_rel{1} * A_rel{2}*A_rel{3} * A_rel{4};
fprintf('Matriz de 0 a 4\n');
disp(simplify(T04))

%%      Trplot

% T01 = A_rel{1};
% T02 = A_rel{1} * A_rel{2};
% T03 = A_rel{1} * A_rel{2} * A_rel{3};
% T04 = A_rel{1} * A_rel{2} * A_rel{3} * A_rel{4};
% 
% T01 = double(T01);
% T02 = double(T02);
% T03 = double(T03);
% T04 = double(T04);
% 
% figure; hold on; grid on; axis equal
% trplot(eye(4), 'frame','0','color','k','length',10)
% trplot(T01, 'frame','1','color','g','length',8)
% trplot(T02, 'frame','2','color','r','length',8)
% trplot(T03, 'frame','3','color','m','length',8)
% trplot(T04, 'frame','4','color','r','length',8)
% view(45,25)


%%          SerialLINK

q_home=[0,0,0,0];
% Criar os 4 elos de rotação
for i = 1:4
    L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', theta_off(i));
end
R = SerialLink(L, 'name', 'Lite6');
R.plot(q_home);



%                 R_PLOT e R.TEACH        definir q para algo numerico

q_home = zeros(1,4); 
figure;

R.plot(q_home)
R.teach(q_home);

%                  R.FKINE                     definir q para algo numerico

q_deg = zeros(1,4); 
q = deg2rad(q_deg);

% cinemática direta do efetuador
T04_num_SE3 = R.fkine(q);

pos = T04_num_SE3.t; 
Rot = T04_num_SE3.R; 

nx = Rot(1,1); ny = Rot(2,1); nz = Rot(3,1);   % 1ª coluna = n
sx = Rot(1,2); sy = Rot(2,2); sz = Rot(3,2);   % 2ª coluna = s
ax = Rot(1,3); ay = Rot(2,3); az = Rot(3,3);   % 3ª coluna = a

roll  = atan2(ny, nx);
pitch = atan2(-nz, nx*cos(roll) + ny*sin(roll));
yaw   = atan2(sz, az);

rpy_deg = rad2deg([roll, pitch, yaw]);

disp('T_0^4 ='); disp(T04_num_SE3);
disp('Posição [m] ='); disp(pos');
disp('RPY [deg] ='); disp(rpy_deg);