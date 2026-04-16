function Q = INVUFACTORY(T)

% Entrada:  T (4x4) pose do efetuador (mm)
% Saída:    Q  soluções em rad [θ1..θ6]

    % ------- parâmetros --------
    off = deg2rad([  0  -90  -90   0   0   0 ]);
    alfa = deg2rad ([ -90 180 90 90 -90 0]);
    d  = [243.3  0     0   227.6   0   61.5];
    a  = [  0   200   87     0     0     0 ];

    R0G = T(1:3,1:3);
    p0G   = T(1:3,4);

    % ------- centro do punho (wrist center) -------
    % z6 no referencial 0 é a 3ª coluna de R06
    pw = p0G - d(6) * R0G(:,3);
    tx = pw(1); ty = pw(2); tz = pw(3);

    % ---------- θ1 ----------
    th1 = atan2(ty, tx); 

    % ---- variáveis c1, s1, p1, p2, K ----
    c1 = cos(th1); s1 = sin(th1);
    l1 = d(1); l2 = a(2);  l3 = a(3);  l4 = d(4);
    p1 = c1*tx + s1*ty; p2 = tz - l1;              

    % ---------- θ3 ----------
    C0 = p1^2 + p2^2 - l2^2 + l3^2 + l4^2;
    C1 = -2*p1*l3 + 2*p2*l4;
    C2 =  2*p1*l4 + 2*p2*l3;
    
    R   = sqrt(C1^2 + C2^2);
    arg = -C0 / R;
    
    if abs(arg) > 1
        Q = []; return;   % fora de alcance
    end
    
    phi    = atan2(C2, C1);
    delta1 = phi + acos(arg);
    delta2 = phi - acos(arg);

    Q = [];

    deltas = [delta1, delta2];

    for i = 1:2
        delta = deltas(i);

        f =  l3*cos(delta) - l4*sin(delta);
        g = -l4*cos(delta) - l3*sin(delta);
    
        % θ2 DH
        th2p = atan2( p1 - f,  p2 - g );
        % θ3 DH
        th3p = th2p - delta;

        % Remover offsets (off2=off3=-pi/2)
        th1f = th1;                 % off1=0
        th2f  = th2p - off(2);
        th3f  = th3p - off(3); 

         % ---------- R36 ----------
        A1  = dh(th1f + off(1), d(1), a(1), alfa(1));
        A2  = dh(th2f  + off(2), d(2), a(2), alfa(2));
        A3  = dh(th3f  + off(3), d(3), a(3), alfa(3));
        A03 = (A1*A2*A3);
        R03 = A03(1:3,1:3);
        R36 = R03.' * R0G;

        r13 = R36(1,3); r23 = R36(2,3); r33 = R36(3,3);
        r31 = R36(3,1); r32 = R36(3,2);

         % ------------------- θ4 ----------
        th4 = atan2(r23, r13);

         % ------------------- θ5 ----------
        th5 = atan2(-(cos(th4)*r13 + sin(th4)*r23), r33);

         % ------------------- θ6 ----------
        th6 = atan2(-r32, r31);

        % ---- off4..6 = 0 ---- por isso podiamos não subtrair o off
        th4f = th4 - off(4);
        th5f = th5 - off(5);
        th6f = th6 - off(6);

       % empilhar (com wrap opcional)
        q = [th1f, th2f, th3f, th4f, th5f, th6f];
        q = mod(q + pi, 2*pi) - pi; 
        Q = [Q; q];
    end
end