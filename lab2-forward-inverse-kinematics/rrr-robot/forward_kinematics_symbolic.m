%   Exercicse 1                            
% Class1

%   ALÍNEA a)

syms l1 l2 l3 l4 l6 real
syms q1 q2 q3 q4 q5 q6 real
q = [q1 q2 q3 q4 q5 q6];

theta_off = deg2rad([  0  -90  -90   0   0   0 ]);
alpha     = deg2rad([ -90  180   90  90 -90   0 ]);
d         = [l1   0    0 l4  0  l6];
a         = [  0   l2   l3   0    0    0 ];

M_DH = sym(zeros(6,4));
for i = 1:6
    M_DH(i,:) = [alpha(i), a(i), d(i), q(i) + theta_off(i)];
end

[A_rel, T06_sym] = MGD_DH(M_DH);       

% Vamos usar para a determinação da posições do punho e a matriz rotaão do punho ao
% griper em IK

T03 =A_rel{1} * A_rel{2} * A_rel{3};  
P04=T03(1:3,4)+d(4)*T03(1:3,3);
fprintf('Matriz posição de 0 ao punho\n');
disp(simplify(P04))

T36=A_rel{4} * A_rel{5} * A_rel{6};
fprintf('Matriz rotação do punho ao griper\n');
disp(simplify(T36(1:3,1:3)))



% Criar os 6 elos de rotação
for i = 1:6
    L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', theta_off(i));
end

R = SerialLink(L, 'name', 'Lite6');
R.display()
