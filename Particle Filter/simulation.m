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
x = [0,0,pi/6]';

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
   u = [1,1]';
   % Update Pose
   linear = (u(2) + u(1))/2;
   omega = (u(2) - u(1))/robot_diameter;
   h = [linear*dt*cos(x(3)),linear*dt*sin(x(3)),omega*dt]';
   x = x + h;
   
   pause(dt);
   %input('Pause')
end