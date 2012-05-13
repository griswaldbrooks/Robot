cla,clc,clear
hold all

axis([-2 2 -2 2])
axis square

% Initial Conditions, radians
th  = [-pi/2;0];
dth = [0;0];

% Parameters
a1 = 1;     % length 1
a2 = 1;     % length 2

% Accumulated Error
acc_error = [0,0]';

% Error vector
e = [];

% State Vector
x = [];

% Time Step
dt = 0.01; 
T = 10;


for t = 0:dt:T
    % Joint Angle Commands
%     th_ref = [sin(4*t),cos(4*t)]';
%     dth_ref = [4*cos(4*t),-4*sin(4*t)]';
%     ddth_ref = [-16*sin(4*t),-16*cos(4*t)]';
    
    th_ref = [pi/2,0]';
    dth_ref = [0,0]';
    ddth_ref = [0,0]';
    
    e = [e,(th-th_ref)];
    acc_error = acc_error + dt*(th - th_ref);

    k1 = dt*calc_x_dot(th,dth,th_ref,dth_ref,ddth_ref,acc_error);
    k2 = dt*calc_x_dot(th + 0.5*k1(1:2),dth + 0.5*k1(3:4),th_ref,dth_ref,ddth_ref,acc_error);
    k3 = dt*calc_x_dot(th + 0.5*k2(1:2),dth + 0.5*k2(3:4),th_ref,dth_ref,ddth_ref,acc_error);
    k4 = dt*calc_x_dot(th + k3(1:2),dth + k3(3:4),th_ref,dth_ref,ddth_ref,acc_error);
    
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    th = th + k(1:2);
    dth = dth + k(3:4);
    
    x = [x;th(1),dth(1),th(2),dth(2)];
    
    figure(1);
    cla
    % Plot target positions
    plot(a1*cos(th_ref(1)), a1*sin(th_ref(1)), 'bo','MarkerSize',5,'MarkerFaceColor','r');
    plot(a1*cos(th_ref(1)) + a2*cos(th_ref(1) + th_ref(2)), a1*sin(th_ref(1)) + a2*sin(th_ref(1) + th_ref(2)),'bo','MarkerSize',5,'MarkerFaceColor','r');
    
    % Plot manipulator arm
    line([0, a1*cos(th(1))],[0, a1*sin(th(1))],'Color','b','LineWidth',3);
    line([a1*cos(th(1)), a1*cos(th(1)) + a2*cos(th(1)+th(2))],[a1*sin(th(1)), a1*sin(th(1)) + a2*sin(th(1)+th(2)) ],'Color','b','LineWidth',3);
    plot(a1*cos(th(1)), a1*sin(th(1)), 'bo','MarkerSize',20,'MarkerFaceColor','k');
    plot(a1*cos(th(1)) + a2*cos(th(1) + th(2)), a1*sin(th(1)) + a2*sin(th(1) + th(2)),'bo','MarkerSize',20,'MarkerFaceColor','k');
    
    
    th_prev = th;
    dth_prev = dth;
    pause(dt)
    %input('Pause')
end

% Plot error
  figure(2);
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
axis([0,dt*length(x),min(min(x)),max(max(x))])
plot(x(:,1),x(:,2),x(:,3),x(:,4))




