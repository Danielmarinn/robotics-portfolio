function e = pose_err(Td, Th)
    % erro de posição (mm) e orientação (rad)
    pd = Td(1:3,4); ph = Th(1:3,4);
    R  = Td(1:3,1:3)' * Th(1:3,1:3);
    ang = acos( max(-1,min(1,(trace(R)-1)/2)) ); % ângulo eixo
    e.pos = norm(ph - pd);
    e.ori = ang;
end

function y = wrapPi(x), y = mod(x+pi, 2*pi) - pi; end
