function U = controller5(theta_ref,dth_ref,ddth_ref,theta_ss,dtheta_ss,acc_error)
% Feedback Linearizing Controller
m1 = 1;     % kg
m2 = 1;     % kg
a1 = 1;     % length 1
a2 = 1;     % length 2
g = 9.81;    % m/s^2

U = [0,0]';
T = [0,0]';

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



U(2) = (J(2,1)-J(2,2)*inv(J(1,2))*J(1,1))*(Kp*(theta_ss(1) - theta_ref(1)) + Ki*acc_error(1) + Kd*(dtheta_ss(1)-dth_ref(1)) + ddth_ref(1)) - J(2,2)*inv(J(1,2))*(F(1,1) + G(1,1)) + F(2,1) + G(2,1);
%T(2) = (J(2,2)-J(2,1)*inv(J(1,1))*J(2,1))*(Kp*(theta_ss(2) - theta_ref(2)) + Ki*acc_error(2) + Kd*(dtheta_ss(2)-dth_ref(2)) + ddth_ref(2)) - J(2,1)*inv(J(1,1))*(F(1,1) + G(1,1)) + F(2,1) + G(2,1);



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

