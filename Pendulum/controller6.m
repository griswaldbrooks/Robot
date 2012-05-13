function U = controller6(theta_ref,dth_ref,ddth_ref,theta_ss,dtheta_ss,acc_error)
% Feedback Linearizing Controller
m1 = 1;     % kg
m2 = 1;     % kg
a1 = 1;     % length 1
a2 = 1;     % length 2
g = 9.81;    % m/s^2
b1 = 0.1;     % dampening 1
b2 = 0.1;     % dampening 2

dt = 0.01;

R = [1/10,0;0,1/10];
Q = [10,0;0,10];

U = [0,0]';
u_step = 1;
u_actions = -100:u_step:100;

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
     
% Dampening
  B = [-b1*dtheta_ss(1); -b2*dtheta_ss(2)];

%%% Find the minimal action now that decreases cost
min_U = [0,0]';
min_cost = 1e6;
for ndx = 1:u_step:length(u_actions)
    action = [0,u_actions(ndx)]';
    next_state = theta_ss + dt*(dt*inv(J)*(-F - G + B + action) + dtheta_ss);
    % e is the error between the goal and the next state, given u
    e = next_state - theta_ref;
    %cost = e'*Q*e + action'*R*action
    cost = e'*Q*e;
    if(cost < min_cost)
        min_cost = cost;
        min_U = action;
    end
end
%%%
cost;
U = min_U;

K = 100*eye(2);
U = -K*(theta_ss - theta_ref)
U(1) = 0;






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