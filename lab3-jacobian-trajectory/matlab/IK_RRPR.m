function q = IK_ex1(alpha, Cx, Cz, r, lg)
% IK_ex1  Cinemática inversa do exercício 1
%   q = [q1; q2; d3; q4]
%
%   alpha  - ângulo na circunferência [rad]
%   Cx,Cz  - centro da circunferência (em cm)
%   r      - raio (em cm)
%   lg     - comprimento da "mão" (distância junta4 -> gripper) [cm]

    % 1) coordenadas do punho W
    xW = Cx + (r - lg)*cos(alpha);
    zW = Cz - (r - lg)*sin(alpha);

    % 2) IK estilo professor
    q1 = 0;
    q2 = atan2(zW, xW);
    d3 = sqrt(xW^2 + zW^2);
    q4 = -alpha - q2;

    q = [q1; q2; d3; q4];
end
