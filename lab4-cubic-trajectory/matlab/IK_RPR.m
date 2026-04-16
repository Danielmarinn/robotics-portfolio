function q = IK_RPR(alpha, Cx, Cy, r, lg)

% 1) Ponto desejado do GRIPPER (círculo do enunciado)
x = Cx + r*cos(alpha);
y = Cy + r*sin(alpha);

% 2) Convenção usada no teu Jacobiano/3(a):
%    psi = -(theta1+theta3) e psi = alpha
%    => theta1 + theta3 = -alpha
phi = -alpha;  % phi = theta1 + theta3

% 3) Retirar o offset lg na direção do tool (orientação phi)
%    [x; y] = [d2*cos(theta1); d2*sin(theta1)] + lg*[cos(phi); sin(phi)]
xP = x - lg*cos(phi);
yP = y - lg*sin(phi);

% 4) Resolver theta1 e d2
theta1 = atan2(yP, xP);
d2     = hypot(xP, yP);   % cm

% 5) Resolver theta3 a partir de phi
theta3 = phi - theta1;

q = [theta1; d2; theta3];
end
