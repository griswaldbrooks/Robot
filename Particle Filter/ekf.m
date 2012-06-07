function [xkp1,Pkp1] = ekf(xk,zk,uk,dt,landmark,Pk)

% Parameters
robot_diameter = 0.3; 
Cv = 0.1*eye(3);
Cw = 0.1;

% Control Input
v = (uk(2) + uk(1))/2;
w = (uk(2) - uk(1))/robot_diameter;

%%% Prediction Phase %%%
% Predict State 
dx = [v*dt*cos(xk(3)),v*dt*sin(xk(3)),w*dt]';
xkp1_p = xk + dx;

% Linearized Plant Model
F = [   1,  0,  -v*dt*sin(xkp1_p(3));
        0,  1,  v*dt*cos(xkp1_p(3));
        0,  0,  1];

% Linearized Sensor Model
H = [   xkp1_p(1)*(xkp1_p(1) - landmark(1)), xkp1_p(2)*(xkp1_p(2) - landmark(2)), 0];

% State Covariance
Pkp1_p = F*Pk*F' + Cv;


%%% Update Phase %%%
% Innovation Covariance
S = H*Pkp1_p*H' + Cw;
% Kalman Gain
K = (Pkp1_p*(H'))/S;

% Innovation
h = sqrt((xkp1_p(1) - landmark(1))^2 + (xkp1_p(2) - landmark(2))^2);
r = zk - h;

% Update State
xkp1 = xkp1_p + K*r;

% Update State Covariance
Pkp1 = (eye(3) - K*H)*Pkp1_p;

