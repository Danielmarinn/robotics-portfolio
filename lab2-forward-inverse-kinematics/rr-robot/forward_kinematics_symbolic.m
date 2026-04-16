    
%   Exercicse 2                            

%   ALÍNEA d)
clc; clear; close all;

syms l1 l2 lg real
syms q1 q2 q3 q4 real
q = [q1 q2 q3 q4]; 
theta_off = deg2rad([  0  0  +90   +90]);
alpha     = deg2rad([ -90  +90   +90  0]);
d         = [-l1  +l2  0  +lg];
a         = [+l2  +l1  0   0];

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