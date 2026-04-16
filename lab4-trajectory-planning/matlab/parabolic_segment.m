function [q, qd, qdd, t_seg] = parabolic_segment(q0, q1, T, a_max, dt)
% Perfil parabólico (triangular ou trapezoidal) entre q0 e q1 em tempo T.
% Garante |qdd| <= a_max.
% q0,q1 em unidades da junta (rad ou cm), a_max nessas unidades/s^2.

N  = floor(T/dt) + 1;
t_seg = linspace(0, T, N).';    % coluna

dq = q1 - q0;
if abs(dq) < 1e-10
    % junta não se mexe
    q   = q0*ones(N,1);
    qd  = zeros(N,1);
    qdd = zeros(N,1);
    return;
end

a_max = abs(a_max);             % magnitude
sgn   = sign(dq);               % sinal do movimento

% aceleração necessária para perfil triangular puro
a_tri = 4*abs(dq) / T^2;

if a_tri <= a_max + 1e-12
    % --------- Perfil triangular (aceleração menor que limite) ---------
    a = sgn*a_tri;
    t1 = T/2;                    % fim da aceleração

    q   = zeros(N,1);
    qd  = zeros(N,1);
    qdd = zeros(N,1);

    for k = 1:N
        tau = t_seg(k);
        if tau <= t1
            % aceleração
            q(k)   = q0 + 0.5*a*tau^2;
            qd(k)  = a*tau;
            qdd(k) = a;
        else
            % desaceleração
            t2   = tau - t1;
            q_mid = q0 + 0.5*a*t1^2;
            v_mid = a*t1;
            q(k)   = q_mid + v_mid*t2 - 0.5*a*t2^2;
            qd(k)  = v_mid - a*t2;
            qdd(k) = -a;
        end
    end
else
    % --------- Perfil trapezoidal (usa a_max) ---------
    a = sgn*a_max;

    % tempos de aceleração/desaceleração (iguais) com a_max
    t1 = T/2 - abs(dq)/(2*a_max*T);  % derivado de area do trapézio
    if t1 < 0
        t1 = 0;
    end
    t2 = T - t1;                     % fim da desaceleração

    v_max = a*t1;                    % velocidade plateau

    q   = zeros(N,1);
    qd  = zeros(N,1);
    qdd = zeros(N,1);

    for k = 1:N
        tau = t_seg(k);
        if tau <= t1
            % aceleração
            q(k)   = q0 + 0.5*a*tau^2;
            qd(k)  = a*tau;
            qdd(k) = a;
        elseif tau <= t2
            % velocidade constante
            q1a = q0 + 0.5*a*t1^2;
            q(k)   = q1a + v_max*(tau - t1);
            qd(k)  = v_max;
            qdd(k) = 0;
        else
            % desaceleração
            t3   = tau - t2;
            q2a  = q0 + 0.5*a*t1^2 + v_max*(t2 - t1);
            q(k)   = q2a + v_max*t3 - 0.5*a*t3^2;
            qd(k)  = v_max - a*t3;
            qdd(k) = -a;
        end
    end
end

% corrigir fim exatamente
q(end)   = q1;
qd(end)  = 0;
qdd(end) = 0;
end

