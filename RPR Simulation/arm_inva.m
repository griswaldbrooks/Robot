function qddot = arm_inva(a,q,qdot,a3)

J = [   -a3*sin(q(1)+q(3)) + q(2)*cos(q(1)),    sin(q(1)),  -a3*sin(q(1)+q(3));
         a3*cos(q(1)+q(3)) + q(2)*sin(q(1)),   -cos(q(1)),  a3*cos(q(1)+q(3));
         0,                                             0,                  0;
         0,                                             0,                  0;
         0,                                             0,                  0;
         1,                                             0,                  1];
         
dJ = [   -a3*cos(q(1)+q(3))*(qdot(1)+qdot(3)) + qdot(2)*cos(q(1)) - q(2)*sin(q(1))*qdot(1),   cos(q(1))*qdot(1),  -a3*cos(q(1)+q(3))*(qdot(1)+qdot(3));
         -a3*sin(q(1)+q(3))*(qdot(1)+qdot(3)) + qdot(2)*sin(q(1)) + q(2)*cos(q(1))*qdot(1),   sin(q(1))*qdot(1),  -a3*sin(q(1)+q(3))*(qdot(1)+qdot(3));
         0,                                             0,                  0;
         0,                                             0,                  0;
         0,                                             0,                  0;
         0,                                             0,                  0];
         
 qddot = J\(a - dJ*qdot);