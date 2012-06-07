% Simulation of a planar differential drive robot localizing
% off a single point landmark with known correspondence and position

cla,clc,clear
hold all

axis([-10 10 -10 10])
axis square
xlabel('X axis, m')
ylabel('Y axis, m')

% Kinematic Parameters (meters)
robot_diameter = 0.3;

% Robot State
% pose = [x,y,\theta]'
% Actual State
x = [0,0,pi/6]';
% Estimated State
xk = x;
% State Covariance
Pk = eye(1);

% Control Inputs
% u = [velocity left, velocity right]
u = [0,0]';

% Landmark State
% landmark = [x,y]'
landmark = [5,5]';

% Frame Scaler
fr_sc = 1;
% Time Step (s)
dt = 0.01;
% Final Time (s)
T = 10;

% Simulation Loop
for t = 0:dt:T
   cla
   % Plot Robot Position
   plot(x(1), x(2), 'bo');
   % Plot Robot Orientation
   line([x(1),fr_sc*cos(x(3)) + x(1)],[x(2),fr_sc*sin(x(3)) + x(2)], 'Color', 'b');
   % Plot Landmark
   plot(landmark(1), landmark(2), 'r+');
   
   % Update Control Input
   u = [3,3.5]';
   % Update Pose
   % Movement noise
   vl = 0.1*rand(1);
   vw = 0.2*rand(1);
   % Motion Model
   linear = ((u(2) + u(1))/2) + vl;
   omega = ((u(2) - u(1))/robot_diameter) + vw;
   dx = [linear*dt*cos(x(3)),linear*dt*sin(x(3)),omega*dt]';
   %dx = [linear*dt*cos(x(3) + omega*dt),linear*dt*sin(x(3) + omega*dt),omega*dt]';
   x = x + dx;
   
   % Measurement Noise
   w = 0.01*rand(1);
   % Measurement Model h(x,m)
   h = sqrt((x(1) - landmark(1))^2 + (x(2) - landmark(2))^2);
   z = h + w;
   
   % EKF
   [xk,Pk] = ekf(xk,z,u,dt,landmark,Pk)
   % Plot Estimated Robot Position
   plot(xk(1), xk(2), 'go');
   % Plot Estimated Robot Orientation
   line([xk(1),fr_sc*cos(xk(3)) + xk(1)],[xk(2),fr_sc*sin(xk(3)) + xk(2)], 'Color', 'g');
   
   pause(dt);
   %input('Pause')
end