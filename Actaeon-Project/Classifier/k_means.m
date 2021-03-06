function clusters = k_means(k,pts)
%%% Cluster structure is
 %%% [c1x1,c1y1,c2x1,c2y1,...,cnx1,cny1;c1x2,c1y2,...,cnx2,cny2;...;c1xm,c1
 %%% ym,...,cnxm,cnym]
clusters = zeros(size(pts,1),2*k);
new_means = zeros(k,2);
old_means = zeros(k,2);
distances = zeros(1,k);

%%% Generate initial means via Random Partition
temp_set = pts;
%%% Place points into clusters, Ensure that each cluster has at least one
%%% point
%pts
for k_ndx = 1:2:(2*k)
    clusters(1,k_ndx:(k_ndx+1)) = temp_set(1,:);
    temp_set(1,:) = [];
end

%%% Randomly place the rest
c_ndx = 2;
while(~isempty(temp_set))
    r_ndx = ceil(k*rand(1));
    r_ndx = 2*r_ndx - 1;
    clusters(c_ndx,r_ndx:(r_ndx+1)) = temp_set(1,:);
    temp_set(1,:) = [];
    c_ndx = c_ndx + 1;
end
%clusters
%input('pause: k_means 29')

%%% Generate initial means, reset clusters
for c_ndx = 1:2:(2*k-1)
    
    x_vect = clusters(:,c_ndx);
    y_vect = clusters(:,c_ndx+1);
    %%% Trim Zeros %%%
    x_vect(all(x_vect==0,2),:) = [];
    y_vect(all(y_vect==0,2),:) = [];
    
    pt_avg_x = mean(x_vect);
    pt_avg_y = mean(y_vect);
 
    new_means((c_ndx+1)/2,:) = [pt_avg_x,pt_avg_y];
    clusters(:,c_ndx:(c_ndx+1)) = zeros(size(clusters,1),2);
end

%new_means
%input('pause: k_means 48')
%%% Start Main Loop
while(any(any(old_means ~= new_means)))
    
    %%% Clear Clusters
    clusters(:,c_ndx:(c_ndx+1)) = zeros(size(clusters,1),2);
    
    %%% Reassign Points
    temp_set = pts;
    c_ndx = 1;
    while (~isempty(temp_set))
        %%% Determine the distances from each point to each mean
        for k_ndx = 1:k
            distances(k_ndx) = sqrt((new_means(k_ndx,1) - temp_set(1,1))^2 + (new_means(k_ndx,2) - temp_set(1,2))^2);
        end
        temp_set(1,:);
        %distances
        %input('pause: k_means 64')
        %%% Arg min %%%
        min_dis = 100000000000000;
        for k_ndx = 1:k
            if distances(k_ndx) < min_dis
                min_k = k_ndx;
                min_dis = distances(k_ndx);
            end
        end
        min_k = 2*min_k -1;
        clusters(c_ndx,(min_k:(min_k+1))) = temp_set(1,:);
        temp_set(1,:) = [];
        c_ndx = c_ndx + 1;
    end
    %clusters
    %input('pause: k_means 80')
    %%% Calculate new means
    old_means = new_means;
    new_means = zeros(k,2);
    for c_ndx = 1:2:(2*k-1) 
        x_vect = clusters(:,c_ndx);
        y_vect = clusters(:,c_ndx+1);
        %%% Trim Zeros %%%
        x_vect(all(x_vect==0,2),:) = [];
        y_vect(all(y_vect==0,2),:) = [];
        
        if isempty(x_vect)
            pt_avg_x = 0;
        else
            pt_avg_x = mean(x_vect);
        end
        if isempty(y_vect)
            pt_avg_y = 0;
        else
            pt_avg_y = mean(y_vect);
        end
 
        new_means((c_ndx+1)/2,:) = [pt_avg_x,pt_avg_y];
    end
    %new_means
    %input('pause: k_means 93')
end





















