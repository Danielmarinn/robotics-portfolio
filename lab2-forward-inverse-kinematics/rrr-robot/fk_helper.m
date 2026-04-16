function T = fk_u_factory(q)

    off = deg2rad([0 -90 -90 0 0 0]);
    alfa = deg2rad([-90 180 90 90 -90 0]);
    d  = [243.3 0 0 227.6 0 61.5];
    a  = [0 200 87 0 0 0];
    
    A1 = dh(q(1)+off(1), d(1), a(1), alfa(1));
    A2 = dh(q(2)+off(2), d(2), a(2), alfa(2));
    A3 = dh(q(3)+off(3), d(3), a(3), alfa(3));
    A4 = dh(q(4)+off(4), d(4), a(4), alfa(4));
    A5 = dh(q(5)+off(5), d(5), a(5), alfa(5));
    A6 = dh(q(6)+off(6), d(6), a(6), alfa(6));
    T = A1*A2*A3*A4*A5*A6;
end
