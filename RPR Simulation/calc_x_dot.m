function x_dot = calc_x_dot(q,dq,q_ref,dq_ref,ddq_ref,acc_error,acc_Fe,wK)
m1 = 4.5;     % kg
m2 = 2;       % kg
m3 = 1;       % kg
a1 = 0.6;     % length 1
a2 = 0.5;     % length 2
a3 = 0.3;     % length 2
g = 9.81;     % m/s^2
b1 = 0.1;     % dampening 1
b2 = 0.1;     % dampening 2
b3 = 0.1;     % dampening 3
c = 0.5;      % Wall drag
wall_x = 0.6; % Wall x location

% Forward Kinematics
    A1 = [  cos(q(1)),     0,      sin(q(1)),      0;
            sin(q(1)),     0,     -cos(q(1)),      0;
            0,             1,              0,      0;
            0,             0,              0,      1];
    
    A2 = [  1,      0,      0,      0;
            0,      0,      1,      0;
            0,     -1,      0,   q(2);
            0,      0,      0,      1];
        
    A3 = [  cos(q(3)),     -sin(q(3)),      0,      a3*cos(q(3));
            sin(q(3)),      cos(q(3)),      0,      a3*sin(q(3));
            0,              0,              1,      0;
            0,              0,              0,      1];
        
    
    A1A2 = A1*A2;
    A1A2A3 = A1A2*A3;

% Jacobian
  J = [   -a3*sin(q(1)+q(3)) + q(2)*cos(q(1)),    sin(q(1)),  -a3*sin(q(1)+q(3));
         a3*cos(q(1)+q(3)) + q(2)*sin(q(1)),   -cos(q(1)),  a3*cos(q(1)+q(3));
         0,                                             0,                  0;
         0,                                             0,                  0;
         0,                                             0,                  0;
         1,                                             0,                  1];

v = J*dq;     
% Generalized Force Vector
  dx = wall_x - A1A2A3(1,4);
  if(dx > 0)
      dx = 0;
      c = 0;
  end
  F = [wK*dx,-c*v(2),0,0,0,0]';



% Dampening
  B = [ -b1*dq(1); 
        -b2*dq(2);
        -b3*dq(3)];
  % Inertia
  M = [ (m2+m3)*q(2)^2 + m3*a3*(1 - 2*q(2)*sin(q(3))),     -m3*a3*cos(q(3)),       m3*a3*(1 - q(2)*sin(q(3)));
                                   -m3*a3*cos(q(3)),                m2 + m3,                 -m3*a3*cos(q(3));
                         m3*a3*(1 - q(2)*sin(q(3))),     -m3*a3*cos(q(3)),                            m3*a3];

% Coriolis and Centripetal
  C = [ 2*q(2)*dq(2)*dq(1)*(m2+m3) + m3*a3*(dq(2)*sin(q(3)) - (dq(2)*sin(q(3)) + q(2)*cos(q(3))*dq(3))*(2*dq(1) + dq(3)));
        m3*a3*sin(q(3))*(dq(1) + dq(3))^2 - (m2+m3)*q(2)*dq(1)^2;
        m3*a3*(q(2)*dq(1)*dq(3)*cos(q(3)) - 2*sin(q(3))*dq(2)*dq(1))];

% Gravity Terms
  G = g*[   (m2+m3)*q(2)*sin(q(1)) + m3*a3*cos(q(1)+q(3));
          -(m2+m3)*cos(q(1));
            m3*a3*cos(q(1)+q(3))];
% Control Input
   U = controller(q_ref,dq_ref,ddq_ref,q,dq,acc_error,F,acc_Fe);
   
   

%            /  dq1
%   x_dot =  |  dq2
%            |  dq3
%            |  ddq1
%            |  ddq2
%            \  ddq3

ddq = (M)\(-C - G + B + U + J'*F);

%input('Pause')

x_dot = [dq;ddq];

