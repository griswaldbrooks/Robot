function [r_pose,xn,yn,landmarks] = classifier(laser_rp,r_pose,xn,yn,map,landmarks)

% t_land = [];
% for l_ndx = 1:4:size(landmarks,1)
%     t_land =  [t_land;landmarks(l_ndx),landmarks(l_ndx+2),landmarks(l_ndx+3),landmarks(l_ndx+4)];
% end
% landmarks = t_land;

%%% Merge Previous Landmarks %%%
%landmarks = merge_landmarks2(landmarks);

%%% Correct for Translation Error %%%
%[r_pose]=correct_trans(r_pose,laser_rp,landmarks);

%%% Generate list of ranges and relative angles
laser_rth = zeros(length(laser_rp),2);
angle_increment = (2*pi)/length(laser_rp);
for index = 1:length(laser_rp)
    angle = index*angle_increment + r_pose(3);
    laser_rth(index,:) = [laser_rp(index),angle];
end

%%% Produce Candiate Point Clusters %%%
vv_pts = parse_scan(laser_rth,r_pose);

for c_ndx = 1:2:size(vv_pts,2)
    plot(vv_pts(:,c_ndx),vv_pts(:,c_ndx+1), 'r+')
end

%%% Generate Line Landmark Hypotheses %%%
current_hypos = generate_hypotheses(vv_pts);
current_hypos

%%% Correct for Orientation Error %%%
%[r_pose,xn,yn] = correct_orient(current_hypos,r_pose,xn,yn);

%[r_pose,xn,yn] =local1(current_hypos,r_pose,xn,yn,laser_rp,landmarks);


%%% DRAW HYPOTHESES %%%
for iter = 1:2:size(current_hypos,1)
    line([current_hypos(iter,1),current_hypos(iter,3)],[current_hypos(iter,2),current_hypos(iter,4)],'Color','r')
end

landmarks = associate_hypotheses(landmarks, current_hypos);

%%% DRAW LANDMARKS %%%
for iter = 1:size(landmarks,1)
    line([landmarks(iter,1),landmarks(iter,3)],[landmarks(iter,2),landmarks(iter,4)],'Color','g')
end

