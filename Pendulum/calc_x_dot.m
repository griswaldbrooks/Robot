function x_dot = calc_x_dot(x_prev,x_dot_prev,th_ref,dth_ref,ddth_ref,acc_error)
m1 = 1;     % kg
m2 = 1;     % kg
a1 = 1;     % length 1
a2 = 1;     % length 2
g = 9.81;    % m/s^2
b1 = 0.1;     % dampening 1
b2 = 0.1;     % dampening 2

th = [x_prev(1);x_prev(2)];
dth = [x_dot_prev(1);x_dot_prev(2)];

% Inertia
  J = [(a1^2)*(m1 + m2) + m2*a2^2 + 2*m2*a1*a2*cos(th(2)), m2*a2^2 + m2*a1*a2*cos(th(2));
        m2*a2^2 + m2*a1*a2*cos(th(2)),                     m2*a2^2];

% Coriolis and Centripetal
  F = [-m2*a1*a2*sin(th(2))*(dth(2))*(2*dth(1) + dth(2)); -m2*a1*a2*sin(th(2))*dth(1)*(dth(1) + 2*dth(2))];

% Gravity Terms
  G = g*[m1*a1*cos(th(1)) + m2*a2*cos(th(1) + th(2)) + m2*a1*cos(th(1)); m2*a2*cos(th(1) + th(2))];

% Dampening
  B = [-b1*dth(1); -b2*dth(2)];
  
% Control Input
   U = controller5(th_ref,dth_ref,ddth_ref,th,dth,acc_error);
   

%            /  dtheta1
%   x_dot =  |  dtheta2
%            |  ddtheta1
%            \  ddtheta2

ddth = inv(J)*(-F - G + B + U);

x_dot = [dth;ddth];
            