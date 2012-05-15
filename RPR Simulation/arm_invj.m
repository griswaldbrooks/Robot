function qdot = arm_invj(v,q,a3)

J = [   -a3*sin(q(1)+q(3)) + q(2)*cos(q(1)),    sin(q(1)),  -a3*sin(q(1)+q(3));
         a3*cos(q(1)+q(3)) + q(2)*sin(q(1)),   -cos(q(1)),  a3*cos(q(1)+q(3));
         0,                                             0,                  0;
         0,                                             0,                  0;
         0,                                             0,                  0;
         1,                                             0,                  1];
         
 qdot = J\v;