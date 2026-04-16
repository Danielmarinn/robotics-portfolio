function Q = INVUFACTORY(T)

    l1=25; l2=10; lg=2;
    theta_off = deg2rad([  0  0  +90   +90]); %#ok<NASGU>
    alpha     = deg2rad([ -90  +90   +90  0]); %#ok<NASGU>
    d         = [-l1  +l2  0  +lg];            %#ok<NASGU>
    a         = [+l2  +l1  0   0];             %#ok<NASGU>

    R0G = T(1:3,1:3);
    p0G = T(1:3,4);

    % ------- centro do punho (wrist center) -------
    pw = p0G - lg * R0G(:,3);
    tx = pw(1); ty = pw(2); tz = pw(3);

    % ====== θ2 ======
    s2 = -(tz + l1)/l1;          % sin(theta2)
    theta2 = asin(s2);
    c2 = cos(theta2);

    r31 = R0G(3,1);
    r32 = R0G(3,2);
    r33 = R0G(3,3);

    eps_sing = 1e-6;

    if abs(s2) < eps_sing
        % ---------------------------
        % CASO SINGULAR: theta2 = 0
        % ---------------------------
        theta2 = 0;
        c2 = 1; s2 = 0;

        % θ1 vem só da posição do wrist
        A = l2 + l1*c2;   % = l2 + l1
        B = l2;
        denom = A^2 + B^2;
        c1 = (A*tx + B*ty) / denom;
        s1 = (A*ty - B*tx) / denom;
        theta1 = atan2(s1, c1);

        % quando s2 = 0, 3ª linha dá:
        % r31 = cos(theta4)
        % r32 = -sin(theta4)
        theta4 = atan2(-r32, r31);

        % theta3 é indiferente nesta pose -> podes pôr 0
        theta3 = 0;

    else
        % ---------------------------
        % CASO GERAL
        % ---------------------------

        % ====== θ3 ======
        c3 = -r33 / s2;

        tmp = (r31^2 + r32^2 - c2^2) / (s2^2);
        if tmp < 0
            tmp = 0;
        end
        s3 = sqrt(tmp);              % ramo "elbow-up"
        theta3 = atan2(s3, c3);

        % ====== θ1 ======
        A = l2 + l1*c2;
        B = l2;
        denom = A^2 + B^2;
        c1 = (A*tx + B*ty) / denom;
        s1 = (A*ty - B*tx) / denom;
        theta1 = atan2(s1, c1);

        % ====== θ4 ======
        D = c2^2 + (s2*s3)^2;
        c4 = (r31*c2 - r32*s2*s3) / D;
        s4 = (-r32*c2 - r31*s2*s3) / D;
        theta4 = atan2(s4, c4);
    end

    Q = [theta1, theta2, theta3, theta4];
end
