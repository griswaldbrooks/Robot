% Simulation of a PRR manipulator, using a Feedback Linearizing Controller.
% The simulation is calculated using RK4

cla,clc,clear
hold all

axis([-2 2 -2 2 -2 2])
axis square
xlabel('X axis, m')
ylabel('Y axis, m')
zlabel('Z axis, m')

% Initial Conditions, radians
q  = [0]';
dq = [0]';

% Parameters
m = 10;
k = 1;

% Accumulated Error
acc_error = [0,0,0]';

% Error vector
e = [];

% State Vector
x = [];

% Time Step
dt = 0.1; 
T = 600;


for t = 0:dt:T
    % Joint Angle Commands
%     th_ref = [sin(4*t),cos(4*t)]';
%     dth_ref = [4*cos(4*t),-4*sin(4*t)]';
%     ddth_ref = [-16*sin(4*t),-16*cos(4*t)]';
    
    q_ref = [0]';
    dq_ref = [0]';
    ddq_ref = [0]';
    
    e = [e,(q-q_ref)];
    acc_error = acc_error + dt*(q - q_ref);

    
    %%% Calculate Dynamic Response %%%
    k1 = dt*calc_x_dot(q,dq,t);
    k2 = dt*calc_x_dot(q + 0.5*k1(1),dq + 0.5*k1(2),t);
    k3 = dt*calc_x_dot(q + 0.5*k2(1),dq + 0.5*k2(2),t);
    k4 = dt*calc_x_dot(q + k3(1),dq + k3(2),t);
    
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    q = q + k(1);
    dq = dq + k(2);
    %%%%%%
    
    x = [x;q(1),dq(1)];
    
    figure(1);
    cla
    % Plot target positions
    plot(q(1),0, 'bo','MarkerSize',5,'MarkerFaceColor','r');
    
    % Plot manipulator arm
    line([-1, q(1)],[0, 0],'Color','b','LineWidth',3);
    plot(q(1),0,'bo','MarkerSize',10,'MarkerFaceColor','k');
    
    %pause(dt)
    %input('Pause')
end

% Plot error
  figure(2);
  cla
  e = e*(180/pi);
  axis([0,dt*length(e),min(min(e)),max(max(e))])
  t = 0:dt:T;
  plot(t,e);
  line([0,dt*length(e)],[0,0],'Color','k','LineStyle','--')
  xlabel('Time, seconds');
  ylabel('Error, degrees');
  title('Feedback Linearizing Controller')
  legend1 = legend('Joint 1','Joint 2');
  set(legend1,'Location','SouthEast');

% Graph phase plot
figure(3);
cla
axis([0,dt*length(x),min(min(x)),max(max(x))])
plot(x(:,1),x(:,2),x(:,3),x(:,4))




