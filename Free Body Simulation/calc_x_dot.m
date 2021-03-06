function x_dot = calc_x_dot(parameters,q,dq,q_ref,dq_ref,ddq_ref,acc_error)

a1 = parameters(1);
a2 = parameters(2);
m1 = parameters(3);
m2 = parameters(4);
g = 9.81;    % m/s^2
b1 = -0.01;     % dampening 1
b2 = -0.01;     % dampening 2
b3 = -0.01;     % dampening 3
floor_z = 0;    % floor location
ceil_z = 5;    % ceiling location
rw_z = -5;    % right wall location
lw_z = 5;    % 
fw_z = -5;    % front wall location
bw_z = 5;    % 
floor_stiffness = 600;
ceil_stiffness = 600;
rw_stiffness = 600; % Right wall
lw_stiffness = 600;
fw_stiffness = 600; % Front wall
bw_stiffness = 600;
c = -2.0;      % Floor drag
ddq = zeros(6,1);

 %%% Kinematics %%%
    A1 = [   cos(q(2))*cos(q(3)),                                   -cos(q(2))*sin(q(3)),                                    sin(q(2)),             q(4);
             sin(q(1))*sin(q(2))*cos(q(3)) + cos(q(1))*sin(q(3)),   -sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)),   -sin(q(1))*cos(q(2)),   q(5);
            -cos(q(1))*sin(q(2))*cos(q(3)) + sin(q(2))*sin(q(3)),    cos(q(1))*sin(q(2))*sin(q(3)) + sin(q(1))*cos(q(3)),    cos(q(1))*cos(q(2)),   q(6);
            0,                                                       0,                                                      0,                     1];

% Jacobian
  J = [eye(3);zeros(3)];
  v = J*dq(4:6);     
  
% Generalized Force Vector
  c_f = c;
  c_c = c;
  dz_f = A1(3,4) - floor_z;
  dz_c = ceil_z - A1(3,4);
  
  c_rw = c;
  c_lw = c;
  dz_lw = A1(1,4) - lw_z
  dz_rw = A1(1,4) - rw_z
  
  c_fw = c;
  c_bw = c;
  dz_fw = A1(2,4) - fw_z;
  dz_bw = A1(2,4) - bw_z;
  
  if(dz_f > 0)
      dz_f = 0;
      c_f = 0;
  end
  if(dz_c > 0)
      dz_c = 0;
      c_c = 0;
  end
  if(dz_rw > 0)
      dz_rw = 0;
      c_rw = 0;
  end
  if(dz_lw < 0)
      dz_lw = 0;
      c_lw = 0;
  end
  if(dz_fw > 0)
      dz_fw = 0;
      c_fw = 0;
  end
  if(dz_bw < 0)
      dz_bw = 0;
      c_bw = 0;
  end
  
  F = [-rw_stiffness*dz_rw - c_rw*v(1) - lw_stiffness*dz_lw + c_lw*v(1);
      -bw_stiffness*dz_bw - c_bw*v(2) - fw_stiffness*dz_fw + c_fw*v(2);
       ceil_stiffness*dz_c - c_c*v(3) - floor_stiffness*dz_f + c_f*v(3);
       0;0;0];

% Inertia
  M = m1*eye(3);

% Coriolis and Centripetal
  C = zeros(3,1);

% Gravity Terms
  G = m1*g*[0,0,1]';

% Dampening
  B = [b1*dq(4),b2*dq(5),b3*dq(6)]';
  
% Control Input
   %U = controller5(th_ref,dth_ref,ddth_ref,th,dth,acc_error);
   U = zeros(3,1);
   

%            /  dtheta1
%   x_dot =  |  dtheta2
%            |  ddtheta1
%            \  ddtheta2

ddq(4:6) = M\(-C - G + B + U + J'*F);
x_dot = [dq;ddq];
            