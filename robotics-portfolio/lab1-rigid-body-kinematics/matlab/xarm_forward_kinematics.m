%                               Exercise2

clc; clear; close all;

% ---- Parâmetros DH ----
offsets = deg2rad([0, -90, -90, 0, 0, 0]);
d = [243.3, 0, 0, 227.6, 0, 61.5];
a = [0, 200, 87, 0, 0, 0];
alpha = deg2rad([-90, 180, 90, 90, -90, 0]);

% ===== Variáveis simbólicas =====
syms t1 t2 t3 t4 t5 t6 real
theta = [t1 t2 t3 t4 t5 t6];

% ===== Matrizes A_i e T_0^i (simbólicas) =====
A = cell(1,6); T = cell(1,6);     %Pré-alocar para matlab correr mais rápido
for i = 1:6
    A{i} = dh_matrix(theta(i), offsets(i), alpha(i), a(i), d(i));
end
T{1} = A{1};
for i = 2:6
    T{i} = T{i-1} * A{i};
end

% definições do gráfico
figure; 
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135, 35);
xlim([-500 500]); ylim([-500 500]); zlim([0 650]);         

frame_len=150;
q = [0 0 0 0 0 0]; 

%Passar de simbólicas para numéricas
T_num = cell(1,6);
for i = 1:6 
    T_num{i} = double(subs(T{i}, theta, q)); 
end

origins = zeros(3,7);
origins(:,1) = [0;0;0];
for i = 1:6 
    origins(:,i+1) = T_num{i}(1:3,4); 
end

% Alinhar as origens
hLine = line(origins(1,:), origins(2,:), origins(3,:), 'LineWidth', 2);

hFrame = gobjects(1,6);
for i = 1:6
    hFrame(i) = trplot(T_num{i}, 'frame', num2str(i), 'color','r','length',frame_len);
end

hO = plot3(0,0,0,'k.','MarkerSize',18);
text(0,0,0,'  O','Color','k','FontSize',12);

% marcador do efetuador
hEE = plot3(origins(1,end), origins(2,end), origins(3,end), 'o', 'MarkerSize', 8, 'LineWidth', 2);

% --- Trajetória: 
delta=90;                    % Escolher o angulo de rotação em graus
N = 100;                      % nº de frames
traj = linspace(0, deg2rad(delta), N);   

% Este ciclo vai rodar cada junta separadamente 90 graus voltando depois de
% cada rotação á posição original
for j=1:5
    for k = 1:N
        q = zeros(1,6);
        q(j) = traj(k);  
    
        T = eye(4);
        for i = 1:6
            Ai = dh_matrix(q(i), offsets(i), alpha(i), a(i), d(i));
            T = T * Ai;
            T_num{i} = T;
        end
    
        % Origens
        for i = 1:6, origins(:,i+1) = T_num{i}(1:3,4); end
    
        % ATUALIZAR handles
        set(hLine, 'XData', origins(1,:), 'YData', origins(2,:), 'ZData', origins(3,:));
        for i = 1:6, set(hFrame(i), 'Matrix', T_num{i}); end
        set(hEE, 'XData', origins(1,end), 'YData', origins(2,end), 'ZData', origins(3,end));
    
        drawnow limitrate;      
        pause(0.05)
    end
end


%%                      FUNÇÕES USADAS

function A = dh_matrix(theta_i, off_i, alpha_i, a_i, d_i)
    A = [ cos(theta_i+off_i), -sin(theta_i+off_i)*cos(alpha_i),  sin(theta_i+off_i)*sin(alpha_i),  a_i*cos(theta_i+off_i);
          sin(theta_i+off_i),  cos(theta_i+off_i)*cos(alpha_i), -cos(theta_i+off_i)*sin(alpha_i),  a_i*sin(theta_i+off_i);
          0,                   sin(alpha_i),                     cos(alpha_i),                     d_i;
          0,                   0,                                0,                                1 ];
end
