function U = controller(q_ref,dq_ref,ddq_ref,q,dq,acc_error)
% Feedback Linearizing Controller
m1 = 4.5;     % kg
m2 = 2;       % kg
m3 = 1;       % kg
a1 = 0.6;     % length 1
a2 = 0.5;     % length 2
a3 = 0.3;     % length 2
g = 9.81;    % m/s^2

MAX_TORQUE = 1000;
ZERO_TORQUE = 0;

wn = 50;
Kp = -wn^2;
Kd = -2*wn;
Ki = 100;

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



U = M*(Kp*(q - q_ref) + Ki*acc_error + Kd*(dq-dq_ref) + ddq_ref) + C + G;

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