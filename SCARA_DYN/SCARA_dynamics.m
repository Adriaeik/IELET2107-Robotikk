function Dx = SCARA_dynamics(t,x,u)
    q = x(1:3);
    Dq = x(4:6);
    
    m11 = 25/4 + 3*cos(q(2));
    m12 = 13/4 + (3/2)*cos(q(2));
    m13 = 0;
    m21 = m12;
    m22 = 13/4;
    m23 = 0;
    m31 = m13;
    m32 = m23;
    m33 = 1;
    M = [m11 m12 m13; m21 m22 m23; m31 m32 m33];
     
    c11 = -(3/2)*sin(q(2))*Dq(2);
    c12 = -(3/2)*sin(q(2))*(Dq(1)+Dq(2));
    c13 = 0;
    c21 = -(3/2)*sin(q(2))*(Dq(1)+Dq(2));
    c22 = -(3/2)*sin(q(2))*Dq(1);
    c23 = 0;
    c31 = 0;
    c32 = 0;
    c33 = 0;
    C = [c11 c12 c13; c21 c22 c23; c31 c32 c33];
    
    g1 = 0;
    g2 = 0;
    g3 = 5;
    
    g = [g1;g2;g3];
    
    DDq = M\(-C*q-g+u(t,q,Dq));
    
    Dx = [Dq;DDq];
end

