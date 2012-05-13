%%% Leg Simulation %%%
cla, clc, clear
hold all
axis equal
axis([0,100,0,100])

Link1 = 26; %mm
Link2 = 26; %mm
P_link = 32; %mm
Joint1 = [50,80,(9/10)*2*pi];
Joint2 = [Link1*cos(Joint1(3)) + Joint1(1),Link1*sin(Joint1(3)) + Joint1(2),-pi/2 + Joint1(3)];
Joint3 = [Link2*cos(Joint2(3)) + Joint2(1),Link2*sin(Joint2(3)) + Joint2(2), Joint2(3)];
P_off = 9; %mm
P_link1 = [(Link1 - P_off)*cos(Joint1(3)) + Joint1(1),(Link1 - P_off)*sin(Joint1(3)) + Joint1(2),-pi/2 + Joint1(3)];
P_link2 = [P_link*cos(P_link1(3)) + P_link1(1),P_link*sin(P_link1(3)) + P_link1(2),P_link1(3)];
Foot_Length = 10; %mm
Foot = [0,0,atan2(Joint3(2) - P_link2(2),Joint3(1) - P_link2(1)) + pi];
Foot(1) = Foot_Length*cos(Foot(3)) + P_link2(1);
Foot(2) = Foot_Length*sin(Foot(3)) + P_link2(2);
dt = 0.1;
for t = 0:dt:30
    cla
    th_off1 = -(sin(t)^2)    
    th_off2 = cos(t) - pi/2;    
    %%% Update Links %%%
    %Joint1 = [50,50,-pi/2];
    Joint1(3) = (1)*(th_off1);
    Joint2 = [Link1*cos(Joint1(3)) + Joint1(1),Link1*sin(Joint1(3)) + Joint1(2),Joint1(3) + th_off2];
    if abs(Joint2(3)) >= (2*pi)
        Joint2(3) = Joint2(3) - floor(Joint2(3)/(2*pi))*2*pi;
    end
    %Joint2

    Joint3 = [Link2*cos(Joint2(3)) + Joint2(1),Link2*sin(Joint2(3)) + Joint2(2), Joint2(3)];
    P_link1 = [(Link1 - P_off)*cos(Joint1(3)) + Joint1(1),(Link1 - P_off)*sin(Joint1(3)) + Joint1(2),Joint1(3) + th_off2];
    P_link2 = [P_link*cos(P_link1(3)) + P_link1(1),P_link*sin(P_link1(3)) + P_link1(2),P_link1(3)];
    Foot(3) = atan2(Joint3(2) - P_link2(2),Joint3(1) - P_link2(1)) + pi;
    Foot(1) = Foot_Length*cos(Foot(3)) + P_link2(1);
    Foot(2) = Foot_Length*sin(Foot(3)) + P_link2(2);
    
    %%% Plot Links %%%
    line([Joint1(1),Joint2(1)],[Joint1(2),Joint2(2)],'Color','k')
    line([Joint2(1),Joint3(1)],[Joint2(2),Joint3(2)],'Color','k')
    line([P_link1(1),P_link2(1)],[P_link1(2),P_link2(2)],'Color','k')
    line([Joint3(1),Foot(1)],[Joint3(2),Foot(2)],'Color','k')
    plot(Joint1(1),Joint1(2),'bo')
    plot(Joint2(1),Joint2(2),'bo')
    plot(Joint3(1),Joint3(2),'bo')
    plot(P_link1(1),P_link1(2),'bo')
    plot(P_link2(1),P_link2(2),'bo')
    
    
    pause(dt)
end
