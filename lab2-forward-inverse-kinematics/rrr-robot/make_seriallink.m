function R = make_seriallink()

    off  = deg2rad([0 -90 -90 0 0 0]);
    alfa = deg2rad([-90 180 90 90 -90 0]);
    d    = [243.3 0 0 227.6 0 61.5];
    a    = [0 200 87 0 0 0];

    L(1)=Link('d',d(1),'a',a(1),'alpha',alfa(1),'offset',off(1));
    L(2)=Link('d',d(2),'a',a(2),'alpha',alfa(2),'offset',off(2));
    L(3)=Link('d',d(3),'a',a(3),'alpha',alfa(3),'offset',off(3));
    L(4)=Link('d',d(4),'a',a(4),'alpha',alfa(4),'offset',off(4));
    L(5)=Link('d',d(5),'a',a(5),'alpha',alfa(5),'offset',off(5));
    L(6)=Link('d',d(6),'a',a(6),'alpha',alfa(6),'offset',off(6));

    R = SerialLink(L,'name','UFactory');
end
