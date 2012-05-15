% Simulation of a PRR manipulator, using a Feedback Linearizing Controller.
% The simulation is calculated using RK4

cla,clc,clear
hold all

axis([-1 1 -0.5 2 -2 2])
axis square
xlabel('X axis, m')
ylabel('Y axis, m')
zlabel('Z axis, m')

% Initial Conditions, radians
q  = [pi/1.5,0.6,0]';
dq = [0,0,0]';

% Parameters
a1 = 0.6;     % length 1, m
a2 = 0.5;     % length 2, m
a3 = 0.3;     % length 3, m

L = 0.6;      % meters
h1 = 1;       % meters
h2 = 0.2;     % meters

wK = 6000;     % Wall stiffness

% Accumulated Error
acc_error = [0,0,0]';

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
    
    %q_ref = [t,sin(t),-t]';
    %dq_ref = [1,cos(t),-1]';
    %ddq_ref = [0,-sin(t),0]';
    
    dx = 0.6 + 0.0067;
    dy = 0.4*sin(t) + 0.6;
    or = 0;
    
    H =  [  cos(or),     -sin(or),      0,      dx;
            sin(or),      cos(or),      0,      dy;
            0,                  0,      1,      0;
            0,                  0,      0,      1]; 
    
    v = [0,0.4*cos(t),0,0,0,0]';
    a = [0,-0.4*sin(t),0,0,0,0]';
        
    q_ref = arm_ik(H,a3);
    dq_ref = arm_invj(v,q_ref,a3);
    ddq_ref = arm_inva(a,q_ref,dq_ref,a3);
    
    e = [e,(q-q_ref)]; 
    acc_error = acc_error + dt*(q - q_ref);
    
    %%% Calculate Dynamic Response %%%
    k1 = dt*calc_x_dot(q,dq,q_ref,dq_ref,ddq_ref,acc_error);
    k2 = dt*calc_x_dot(q + 0.5*k1(1:3),dq + 0.5*k1(4:6),q_ref,dq_ref,ddq_ref,acc_error);
    k3 = dt*calc_x_dot(q + 0.5*k2(1:3),dq + 0.5*k2(4:6),q_ref,dq_ref,ddq_ref,acc_error);
    k4 = dt*calc_x_dot(q + k3(1:3),dq + k3(4:6),q_ref,dq_ref,ddq_ref,acc_error);
    
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    q = q + k(1:3);
    dq = dq + k(4:6);
    %%%%%%

    %%% Calculate Forward Kinematics %%%
    A1 = [  cos(q(1)),     0,      sin(q(1)),      0;
            sin(q(1)),     0,     -cos(q(1)),      0;
            0,             1,              0,      0;
            0,             0,              0,      1];
    
    A2 = [  1,      0,      0,      0;
            0,      0,      1,      0;
            0,     -1,      0,   q(2);
            0,      0,      0,      1];
        
    A3 = [  cos(q(3)),     -sin(q(3)),      0,      a3*cos(q(3));
            sin(q(3)),      cos(q(3)),      0,      a3*sin(q(3));
            0,              0,              1,      0;
            0,              0,              0,      1];
        
    
    A1A2 = A1*A2;
    A1A2A3 = A1A2*A3;
    
    A1_r = [  cos(q_ref(1)),     0,      sin(q_ref(1)),      0;
            sin(q_ref(1)),     0,     -cos(q_ref(1)),      0;
            0,                 1,              0,      0;
            0,                 0,              0,      1];
    
    A2_r = [  1,      0,      0,      0;
            0,      0,      1,      0;
            0,     -1,      0,   q_ref(2);
            0,      0,      0,      1];
        
    A3_r = [  cos(q_ref(3)),     -sin(q_ref(3)),      0,      a3*cos(q_ref(3));
            sin(q_ref(3)),      cos(q_ref(3)),      0,      a3*sin(q_ref(3));
            0,                              0,      1,      0;
            0,                              0,      0,      1];
        
    A1A2_r = A1_r*A2_r;
    A1A2A3_r = A1A2_r*A3_r;
    %%%%%%
    

    
    x = [x;q(1),dq(1),q(2),dq(2),q(3),dq(3)];
    
    figure(1);
    cla
    % Draw wall
    line([L,L],[0,h1+1],'Color','k','LineWidth',2);
    % Draw floor
    line([-L,L],[0,0],'Color','k','LineWidth',2);
    % Plot target endpoints
    plot(L,h1,'mo','MarkerSize',10);
    plot(L,h2,'mo','MarkerSize',10);
    
    % Plot target positions
    plot3(A1_r(1,4), A1_r(2,4), A1_r(3,4), 'bo','MarkerSize',5,'MarkerFaceColor','r');
    plot3(A1A2_r(1,4), A1A2_r(2,4), A1A2_r(3,4),'bo','MarkerSize',5,'MarkerFaceColor','r');
    plot3(A1A2A3_r(1,4), A1A2A3_r(2,4), A1A2A3_r(3,4),'bo','MarkerSize',5,'MarkerFaceColor','r');
    
    % Plot manipulator arm
    line([0, A1(1,4)],[0, A1(2,4)],[0, A1(3,4)],'Color','k','LineWidth',3);
    line([A1(1,4), A1A2(1,4)],[A1(2,4), A1A2(2,4)],[A1(3,4), A1A2(3,4)],'Color','k','LineWidth',3);
    line([A1A2(1,4), A1A2A3(1,4)],[A1A2(2,4), A1A2A3(2,4)],[A1A2(3,4), A1A2A3(3,4)],'Color','k','LineWidth',3);
    
    % Plot applied force
    
    line([0,-((wK/100)*(L - A1A2A3(1,4)))],[-0.25,-0.25],'Color','r','LineWidth',5);
    
    % Plot frames
    
    % Frame scaler
    f_sc = 0.1;
    % Line width
    l_wi = 4;
    % Vectors that grab certain parts of the Transformation Matrix
    pp = [0,0,0,1]';
    px = [1,0,0,0]';
    py = [0,1,0,0]';
    pz = [0,0,1,0]';
    
    p_eff = H(1:3,4);
    x_feff = f_sc*H(1:3,1) + p_eff;
    y_feff = f_sc*H(1:3,2) + p_eff;
    z_feff = f_sc*H(1:3,3) + p_eff;
    
    % Plot target frame
    line([p_eff(1),x_feff(1)],[p_eff(2),x_feff(2)],[p_eff(3),x_feff(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p_eff(1),y_feff(1)],[p_eff(2),y_feff(2)],[p_eff(3),y_feff(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p_eff(1),z_feff(1)],[p_eff(2),z_feff(2)],[p_eff(3),z_feff(3)], 'Color', 'b', 'LineWidth', l_wi);
    
    p0 = pp;
    p1 = A1*pp;
    p2 = A1*A2*pp;
    p3 = A1*A2*A3*pp;
    
    x0 = f_sc*px + p0;
    y0 = f_sc*py + p0;
    z0 = f_sc*pz + p0;
    
    x1 = f_sc*A1*px + p1;
    y1 = f_sc*A1*py + p1;
    z1 = f_sc*A1*pz + p1;
    
    x2 = f_sc*A1*A2*px + p2;
    y2 = f_sc*A1*A2*py + p2;
    z2 = f_sc*A1*A2*pz + p2;
    
    x3 = f_sc*A1*A2*A3*px + p3;
    y3 = f_sc*A1*A2*A3*py + p3;
    z3 = f_sc*A1*A2*A3*pz + p3;
    % Frame zero
    line([p0(1),x0(1)],[p0(2),x0(2)],[p0(3),x0(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p0(1),y0(1)],[p0(2),y0(2)],[p0(3),y0(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p0(1),z0(1)],[p0(2),z0(2)],[p0(3),z0(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame one
    line([p1(1),x1(1)],[p1(2),x1(2)],[p1(3),x1(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p1(1),y1(1)],[p1(2),y1(2)],[p1(3),y1(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p1(1),z1(1)],[p1(2),z1(2)],[p1(3),z1(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame two
    line([p2(1),x2(1)],[p2(2),x2(2)],[p2(3),x2(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p2(1),y2(1)],[p2(2),y2(2)],[p2(3),y2(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p2(1),z2(1)],[p2(2),z2(2)],[p2(3),z2(3)], 'Color', 'b', 'LineWidth', l_wi);
    % Frame three
    line([p3(1),x3(1)],[p3(2),x3(2)],[p3(3),x3(3)], 'Color', 'r', 'LineWidth', l_wi);
    line([p3(1),y3(1)],[p3(2),y3(2)],[p3(3),y3(3)], 'Color', 'g', 'LineWidth', l_wi);
    line([p3(1),z3(1)],[p3(2),z3(2)],[p3(3),z3(3)], 'Color', 'b', 'LineWidth', l_wi);
    
    % Plot masses
    plot3(A1(1,4), A1(2,4), A1(3,4), 'bo','MarkerSize',10,'MarkerFaceColor','k');
    plot3(A1A2(1,4), A1A2(2,4), A1A2(3,4),'bo','MarkerSize',10,'MarkerFaceColor','k');
    plot3(A1A2A3(1,4), A1A2A3(2,4), A1A2A3(3,4),'bo','MarkerSize',10,'MarkerFaceColor','k');
    
    
    
    
    pause(dt)
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




