function [M, Mf] = MGD_DH(M_DH)
    n = size(M_DH, 1);        
    M  = cell(1, n);
    Mf = sym(eye(4));   % matriz homogénea identidade, simbólica

    for i = 1:n
        alpha    = M_DH(i,1);
        a        = M_DH(i,2);
        d        = M_DH(i,3);
        thetaoff = M_DH(i,4);

        ca = cos(alpha);   sa = sin(alpha);
        ct = cos(thetaoff); st = sin(thetaoff);

        Ai = [ ct, -st*ca,  st*sa, a*ct;
               st,  ct*ca, -ct*sa, a*st;
               0 ,    sa ,    ca ,   d ;
               0 ,    0  ,    0  ,   1 ];

        M{i} = Ai;
        fprintf('A{%d} = ^%dA_%d =\n', i, i-1, i);
        disp(Ai);

        % acumula para dar ^0T_i
        Mf = Mf * Ai;
    end

    fprintf('Matriz final Mf = T0%d:\n', n);
    disp(Mf);
end
