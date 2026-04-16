function J = J_RPR(q_a, lg)
% J_ex1  Jacobiano 3x3 no referencial base para o Ex1
%   q_a = [q2; d3; q4]
%   lg  = comprimento da "mão" (cm)

    q2 = q_a(1);
    d3 = q_a(2);
    q4 = q_a(3);

    c2  = cos(q2);  s2  = sin(q2);
    c24 = cos(q2 + q4);
    s24 = sin(q2 + q4);

    J = [ -d3*s2 - lg*s24,  c2,       -lg*s24;
           d3*c2 + lg*c24,  s2,        lg*c24;
          -1,               0,        -1     ];
end