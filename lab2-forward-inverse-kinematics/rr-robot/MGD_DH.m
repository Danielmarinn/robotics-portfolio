

function [M, MF] = MGD_DH(PJ_DH)
    n = size(PJ_DH, 1);        
    M  = cell(1, n);
    MF = sym(eye(4));   % matriz homogénea identidade, simbólica

    for i = 1:n
        alpha    = PJ_DH(i,1);
        a        = PJ_DH(i,2);
        d        = PJ_DH(i,3);
        thetaoff = PJ_DH(i,4);

        ca = cos(alpha);   sa = sin(alpha);
        ct = cos(thetaoff); st = sin(thetaoff);

        Ai = [ ct, -st*ca,  st*sa, a*ct;
               st,  ct*ca, -ct*sa, a*st;
               0 ,    sa ,    ca ,   d ;
               0 ,    0  ,    0  ,   1 ];

        M{i} = Ai;
        MF   = MF * Ai;
    end
end