cla,clc,clear
hold all

axis([-5 5 -5 5 -0.1 5])
xlabel('X axis, m')
ylabel('Y axis, m')
zlabel('Z axis, m')
axis square

% Initial Conditions, radians, meters
% [th_x, th_y, th_z, dx, dy, dz]
q  = [0,0,0,0,0,2.5]';
dq = [0,0,0,0,0,-20]';

% Parameters
m1 = 0.10;     % kg
m2 = 1;     % kg
a1 = 1;     % length 1
a2 = 1;     % length 2

parameters = [a1,a2,m1,m2];

% Accumulated Error
acc_error = [0,0,0,0,0,0]';

% Time Step
dt = 0.01; 
T = 20;

% Error vector
e = [];

% State Vector
x = [];

% Frame scaler
f_sc = 1;
% Line width
l_wi = 4;


for t = 0:dt:T
    % Joint Angle Commands
%     q_ref = [sin(4*t),cos(4*t)]';
%     dq_ref = [4*cos(4*t),-4*sin(4*t)]';
%     ddq_ref = [-16*sin(4*t),-16*cos(4*t)]';
    
    q_ref =     [0,0,0,0,0,0]';
    dq_ref =    [0,0,0,0,0,0]';
    ddq_ref =   [0,0,0,0,0,0]';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% Kinematics %%%
    A1 = [   cos(q(2))*cos(q(3)),                                   -cos(q(2))*sin(q(3)),                                    sin(q(2)),             q(4);
             sin(q(1))*sin(q(2))*cos(q(3)) + cos(q(1))*sin(q(3)),   -sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(3)),   -sin(q(1))*cos(q(2)),   q(5);
            -cos(q(1))*sin(q(2))*cos(q(3)) + sin(q(2))*sin(q(3)),    cos(q(1))*sin(q(2))*sin(q(3)) + sin(q(1))*cos(q(3)),    cos(q(1))*cos(q(2)),   q(6);
            0,                                                       0,                                                      0,                     1];
    A2 = [  1, 0, 0, a1;
            0, 1, 0,  0;
            0, 0, 1,  0;
            0, 0, 0,  1];
        
%     A1A2 =
%  
%     [                           cos(ry)*cos(rz),                          -cos(ry)*sin(rz),          sin(ry),                             dx + dxp*cos(ry)*cos(rz)]
%     [ cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry), cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz), -cos(ry)*sin(rx), dy + dxp*(cos(rx)*sin(rz) + cos(rz)*sin(rx)*sin(ry))]
%     [ sin(ry)*sin(rz) - cos(rx)*cos(rz)*sin(ry), cos(rz)*sin(rx) + cos(rx)*sin(ry)*sin(rz),  cos(rx)*cos(ry), dz + dxp*(sin(ry)*sin(rz) - cos(rx)*cos(rz)*sin(ry))]
%     [                                         0,                                         0,                0,                                                    1]
 
        
    A1_r = [ cos(q_ref(2))*cos(q_ref(3)),                                               -cos(q_ref(2))*sin(q_ref(3)),                                                sin(q_ref(2)),                 q_ref(4);
             sin(q_ref(1))*sin(q_ref(2))*cos(q_ref(3)) + cos(q_ref(1))*sin(q_ref(3)),   -sin(q_ref(1))*sin(q_ref(2))*sin(q_ref(3)) + cos(q_ref(1))*cos(q_ref(3)),   -sin(q_ref(1))*cos(q_ref(2)),   q_ref(5);
            -cos(q_ref(1))*sin(q_ref(2))*cos(q_ref(3)) + sin(q_ref(2))*sin(q_ref(3)),    cos(q_ref(1))*sin(q_ref(2))*sin(q_ref(3)) + sin(q_ref(1))*cos(q_ref(3)),    cos(q_ref(1))*cos(q_ref(2)),   q_ref(6);
             0,                                                       0,                                                      0,                     1];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% Calculate Dynamic Response %%%
    k1 = dt*calc_x_dot(parameters, q,dq,q_ref,dq_ref,ddq_ref,acc_error);
    k2 = dt*calc_x_dot(parameters, q + 0.5*k1(1:6),dq + 0.5*k1(7:12),q_ref,dq_ref,ddq_ref,acc_error);
    k3 = dt*calc_x_dot(parameters, q + 0.5*k2(1:6),dq + 0.5*k2(7:12),q_ref,dq_ref,ddq_ref,acc_error);
    k4 = dt*calc_x_dot(parameters, q + k3(1:6),dq + k3(7:12),q_ref,dq_ref,ddq_ref,acc_error);
    
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    q = q + k(1:6);
    dq = dq + k(7:12);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Joint Errors
    e = [e,(q-q_ref)];
    acc_error = acc_error + dt*(q - q_ref);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% States %%%
    x = [x;q(1),dq(1),q(2),dq(2)];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    figure(1);
    cla
    %%% Plot target positions %%%
    plot3(A1_r(1,4),A1_r(2,4),A1_r(3,4), 'bo','MarkerSize',5,'MarkerFaceColor','r');
    %plot(a1*cos(q_ref(1)) + a2*cos(q_ref(1) + q_ref(2)), a1*sin(q_ref(1)) + a2*sin(q_ref(1) + q_ref(2)),'bo','MarkerSize',5,'MarkerFaceColor','r');
    
    % Calculate target frame
    p_r = A1_r(1:3,4);
    x_fr = f_sc*A1_r(1:3,1) + p_r;
    y_fr = f_sc*A1_r(1:3,2) + p_r;
    z_fr = f_sc*A1_r(1:3,3) + p_r;
    
    % Plot target frame
    line([p_r(1),x_fr(1)],[p_r(2),x_fr(2)],[p_r(3),x_fr(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p_r(1),y_fr(1)],[p_r(2),y_fr(2)],[p_r(3),y_fr(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p_r(1),z_fr(1)],[p_r(2),z_fr(2)],[p_r(3),z_fr(3)], 'Color', 'b', 'LineWidth', l_wi);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%% Plot manipulator arm %%%
    p0 = [0,0,0]';
    x0 = f_sc*[1,0,0]' + p0;
    y0 = f_sc*[0,1,0]' + p0;
    z0 = f_sc*[0,0,1]' + p0;
    
    p1 = A1(1:3,4);
    x1 = f_sc*A1(1:3,1) + p1;
    y1 = f_sc*A1(1:3,2) + p1;
    z1 = f_sc*A1(1:3,3) + p1;
    
    % Plot frames
    % Frame zero
    line([p0(1),x0(1)],[p0(2),x0(2)],[p0(3),x0(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p0(1),y0(1)],[p0(2),y0(2)],[p0(3),y0(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p0(1),z0(1)],[p0(2),z0(2)],[p0(3),z0(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame one
    line([p1(1),x1(1)],[p1(2),x1(2)],[p1(3),x1(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p1(1),y1(1)],[p1(2),y1(2)],[p1(3),y1(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p1(1),z1(1)],[p1(2),z1(2)],[p1(3),z1(3)], 'Color', 'b', 'LineWidth', l_wi);
    
    %line([0, a1*cos(q(1))],[0, a1*sin(q(1))],'Color','b','LineWidth',3);
    %line([a1*cos(q(1)), a1*cos(q(1)) + a2*cos(q(1)+q(2))],[a1*sin(q(1)), a1*sin(q(1)) + a2*sin(q(1)+q(2)) ],'Color','b','LineWidth',3);
    plot3(A1(1,4),A1(2,4),A1(3,4), 'bo','MarkerSize',0.5*m1*10,'MarkerFaceColor','k');
    %plot(a1*cos(q(1)) + a2*cos(q(1) + q(2)), a1*sin(q(1)) + a2*sin(q(1) + q(2)),'bo','MarkerSize',0.5*m2*10,'MarkerFaceColor','k');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
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




