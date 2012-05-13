cla,clc,clear
hold all

axis([-1.5 1.5 -1.5 1.5])
axis square
% Initial Conditions, radians
th = pi;
dth = 0.10;

% Parameters
I = 1;      % Inertial
m = 1;      % kg
g = 9.8;    % m/s^2
l = 1;      % length
T = 0;      % external torque
B = 0.8;      % dampening

% Time Step
dt = 0.01;  

for t = 0:dt:50
    cla
    th = th + dt.*dth;
    dth = dth + dt.*(((-m.*g.*l)./I).*sin(th) - B.*dth + (T./I)); 
    line([0, l.*sin(th)],[0, -l.*cos(th)],'Color','b');
    plot(l.*sin(th), -l.*cos(th),'bo');
    pause(dt)
end

