function U = controller4(theta_ref,dth_ref,ddth_ref,theta_ss,dtheta_ss,acc_error)
% Feedback Linearizing Controller
m1 = 1;     % kg
m2 = 1;     % kg
a1 = 1;     % length 1
a2 = 1;     % length 2
g = 9.81;    % m/s^2

MAX_TORQUE = 1000;
ZERO_TORQUE = 0;

Kp = -40;
Ki = -5;
Kd = -10;

% Inertia
  J = [(a1^2)*(m1 + m2) + m2*a2^2 + 2*m2*a1*a2*cos(theta_ss(2)), m2*a2^2 + m2*a1*a2*cos(theta_ss(2));
        m2*a2^2 + m2*a1*a2*cos(theta_ss(2)),                     m2*a2^2];

% Coriolis and Centripetal
  F = [-m2*a1*a2*sin(theta_ss(2))*(dtheta_ss(2))*(2*dtheta_ss(1) + dtheta_ss(2)); 
       -m2*a1*a2*sin(theta_ss(2))*dtheta_ss(1)*(dtheta_ss(1) + 2*dtheta_ss(2))];

% Gravity Terms
  G = g*[m1*a1*cos(theta_ss(1)) + m2*a2*cos(theta_ss(1) + theta_ss(2)) + m2*a1*cos(theta_ss(1)); 
         m2*a2*cos(theta_ss(1) + theta_ss(2))];



U = J*(Kp*(theta_ss - theta_ref) + Ki*acc_error + Kd*(dtheta_ss-dth_ref) + ddth_ref) + F + G;

if(U(1) > MAX_TORQUE)
    U(1) = MAX_TORQUE;
elseif(U(1) < -MAX_TORQUE)
    U(1) = -MAX_TORQUE;
end

if(U(2) > MAX_TORQUE)
    U(2) = MAX_TORQUE;
elseif(U(2) < -MAX_TORQUE)
    U(2) = -MAX_TORQUE;
end

if(isnan(U(1)))
    U(1) = ZERO_TORQUE;
elseif(isinf(U(1)))
    U(1) = ZERO_TORQUE;
end

if(isnan(U(2)))
    U(2) = ZERO_TORQUE;
elseif(isinf(U(2)))
    U(2) = ZERO_TORQUE;
end