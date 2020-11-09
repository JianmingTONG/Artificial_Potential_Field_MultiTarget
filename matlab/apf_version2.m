clc, clear all;

% load apf_problem.mat
load apf_paper2.mat
height = size(map,1);
width  = size(map,2);

% begin=[2;2];

% begin =[74; 115];
begin =[390; 40];
% begin =[20; 80];

% over=[14 14;5 14];
% over=[14 6 14 ;14 14 2];

inflation_radius = 5;
% obstacle=[0 0 0 0 0 0 0 0 0 0 0  0  0  0  0  0   15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 1  2  3  4  5  6  7  8  9  10 11 12 13 14   1 2 3 4 5 6 7 8 9 10 11 12 13 14   5 5 5 5 5 5 5 5 5  5  5  5  10 10 10 10 10 10 10 10; 
%           0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15  0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 0 0 0 0 0 0 0 0 0  0  0  0  0  0   2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6  9  13];
% obstacle=[1 1 1 1 1 1 1 1 1 1  1  1  1  1  1     2  3  4  5  6  7  8  9  10 11 12 13 14    2  3  4  5  6  7  8  9  10 11 12 13 14 15     15 15 15 15 15 15 15 15 15 15 15 15 15 15     5 5 5 5 5 5 5 5 5  5  5  5   10 10 10 10 10 10 10 10; 
%           1 2 3 4 5 6 7 8 9 10 11 12 13 14 15    15 15 15 15 15 15 15 15 15 15 15 15 15    1  1  1  1  1  1  1  1  1  1  1  1  1   1     2  3  4  5  6  7  8  9  10 11 12 13 14 15     1 2 3 4 5 6 7 8 9  10 11 12  14 13 12 11 10 9  8  7];

obstacle = []; 
target = [];
for i = 3 : size(map, 1) - 2
    for j = 3 : size(map, 2) - 2
        if(map(i,j) == 100)
            obstacle = [obstacle, [i;j]];
        end
        if(map(i,j)==-1)
            temp = 0;
            if(map(i+1, j)==0)
                temp1 = 0;
                temp1 = temp1 + (map(i+2, j)==0);
                temp1 = temp1 + (map(i+1, j+1)==0);
                temp1 = temp1 + (map(i+1, j-1)==0);
                temp = temp   + (temp1>0);
            end
            
            if(map(i, j+1)==0)
                temp1 = 0;
                temp1 = temp1 + (map(i, j+2)==0);
                temp1 = temp1 + (map(i+1, j+1)==0);
                temp1 = temp1 + (map(i-1, j+1)==0);
                temp = temp   + (temp1>0);
            end
           
            if(map(i-1, j)==0)
                temp1 = 0;
                temp1 = temp1 + (map(i-1, j+1)==0);
                temp1 = temp1 + (map(i-1, j-1)==0);
                temp1 = temp1 + (map(i-2, j)==0);
                temp = temp   + (temp1>0);
            end
            
            if(map(i, j-1)==0)
                temp1 = 0;
                temp1 = temp1 + (map(i, j-2)==0);
                temp1 = temp1 + (map(i+1, j-1)==0);
                temp1 = temp1 + (map(i-1, j-1)==0);
                temp  = temp  + (temp1>0);
            end
            if(temp > 0)
                target = [target, [i;j]];
            end
        end
    end
end

%%%%%%%%%%%%%%%%%% Remove targets whose distance to the nearest obstacle <
%%%%%%%%%%%%%%%%%% inflation radius.
targets = [];
for i = 1: size(target,2)
    temp = 0;
    for j = 1 : size(obstacle, 2)
       dis = abs(target(1,i)-obstacle(1,j)) ;
       dis = dis + abs(target(2,i)-obstacle(2,j)) ;
       if(dis < inflation_radius)
           temp =1;
           break;
       end
    end
    if(temp == 1)
        continue;
    end
    targets = [targets, target(:,i)];
end

%%%%%%%%%%%%%%%%%% New Solution: Cluster Targets into centers; 
%%%%%%%%%%%%%%%%%% InfoGain = the number of targets in each cluster.
%%%%%%%%%%%%%%%%%% Note: the K_repu coefficient should be set as 4;


%%%%%%%% Original Version1
% targets_process = targets;
% cluster_center = cell(0);
% idx_cell = 1;
% while(size(targets_process, 2) > 0)
%     idx = 1;
%     target_cluster = targets_process(:,1);
%     targets_process(:,1) = [];
%     while (idx < size(targets_process,2))
%         condition = 1;
%         for i = 1: size(target_cluster,2)
%            if(abs(targets_process(1, idx) - target_cluster(1, i)) + abs(targets_process(2, idx) - target_cluster(2, i)) < 3)
%                target_cluster = [target_cluster, targets_process(:, idx)];
%                targets_process(:, idx) = [];
%                condition = 0;
%                break;
%            end
%         end
%         if(condition)
%            idx = idx + 1; 
%         end
%     end
%     cluster_center{1, idx_cell} = target_cluster;
%     idx_cell = idx_cell + 1;
% end
% 
% num_cluster = size(cluster_center, 2);
% center = zeros(2, num_cluster);
% infoGain_cluster = zeros(1, num_cluster);
% for i = 1: num_cluster
%     infoGain_cluster(i) = size(cluster_center{1,i}, 2);
%     for j = 1: infoGain_cluster(i)
%        center(1,i) =  center(1,i) +  cluster_center{1,i}(1,j);
%        center(2,i) =  center(2,i) +  cluster_center{1,i}(2,j);
%     end
%     center(1,i) = center(1,i) / infoGain_cluster(i) ;
%     center(2,i) = center(2,i) / infoGain_cluster(i) ;
%     min_dis = 100;
%     min_idx = 0;
%     for j = 1: size(targets,2)
%         dis = abs(center(1,i) - targets(1,j)) +  abs(center(2,i) - targets(2,j));
%         if(dis < min_dis)
%            min_dis = dis;
%            min_idx= j;
%         end
%     end
%     center(:,i) = targets(:,min_idx);
% end
%%%%%%%% Original Version1 end

% %%%%%%%% Original Version2 start loop from back to start
% targets_process = targets;
% cluster_center = cell(0);
% idx_cell = 1;
% while(size(targets_process, 2) > 0)
%     idx = 1;
%     target_cluster = targets_process(:, end);
%     targets_process(:,end) = [];
%     for idx =  size(targets_process,2): -1:1
%         for i = 1: size(target_cluster,2)
%            if(abs(targets_process(1, idx) - target_cluster(1, i)) + abs(targets_process(2, idx) - target_cluster(2, i)) < 3)
%                target_cluster = [target_cluster, targets_process(:, idx)];
%                targets_process(:, idx) = [];
%                break;
%            end
%         end
%     end
%     
%     cluster_center{1, idx_cell} = target_cluster;
%     idx_cell = idx_cell + 1;
%     
% end
% 
% num_cluster = size(cluster_center, 2);
% center = zeros(2, num_cluster);
% infoGain_cluster = zeros(1, num_cluster);
% for i = 1: num_cluster
%     infoGain_cluster(i) = size(cluster_center{1,i}, 2);
%     for j = 1: infoGain_cluster(i)
%        center(1,i) =  center(1,i) +  cluster_center{1,i}(1,j);
%        center(2,i) =  center(2,i) +  cluster_center{1,i}(2,j);
%     end
%     center(1,i) = center(1,i) / infoGain_cluster(i) ;
%     center(2,i) = center(2,i) / infoGain_cluster(i) ;
%     min_dis = 100;
%     min_idx = 0;
%     for j = 1: size(targets,2)
%         dis = abs(center(1,i) - targets(1,j)) +  abs(center(2,i) - targets(2,j));
%         if(dis < min_dis)
%            min_dis = dis;
%            min_idx= j;
%         end
%     end
%     center(:,i) = targets(:,min_idx);
% end
%%%%%%%% Original Version2 ends


%%%%%%%% New Version4
targets_process = targets;
center = [];
infoGain_cluster = [];
while(size(targets_process, 2) > 0)
    idx = 1;
    target_cluster = targets_process(:, end);
    targets_process(:,end) = [];
    condition = 1;
    while(condition)
        condition = 0;
        for idx =  size(targets_process,2): -1:1
            for i = 1: size(target_cluster,2)
               if(abs(targets_process(1, idx) - target_cluster(1, i)) + abs(targets_process(2, idx) - target_cluster(2, i)) < 3)
                   target_cluster = [target_cluster, targets_process(:, idx)];
                   targets_process(:, idx) = [];
                   condition= 1;
                   break;
               end
            end
        end
    end
    center_temp = zeros(2,1);
    num_ = size(target_cluster,2);
    for j = 1: num_
       center_temp(1) =  center_temp(1) +  target_cluster(1,j);
       center_temp(2) =  center_temp(2) +  target_cluster(2,j);
    end
    
    center_temp(1) = center_temp(1) / num_;
    center_temp(2) = center_temp(2) / num_;
    
    min_dis = 100;
    min_idx = 0;
    
    for j = 1: num_
        dis = abs(center_temp(1) - target_cluster(1,j)) +  abs(center_temp(2) - target_cluster(2,j));
        if(dis < min_dis)
           min_dis = dis;
           min_idx= j;
        end
    end
    center = [center, target_cluster(:,min_idx)];  
    infoGain_cluster =[infoGain_cluster ,num_ ];
end
%%%%%%%%%%%%%%%%%% Version 4 ends



%%%%%%%%%%%%%%%%%% Display targets and obstacles
figure;
plot(begin(1), begin(2),'*b','MarkerSize', 1);
axis([0 height 0 width]);
hold on;
plot(targets(1,:), targets(2,:), '*r', 'MarkerSize', 1);

plot(obstacle(1,:), obstacle(2,:), '*k', 'MarkerSize', 1);

% for i=1:size(obstacle, 2)
%     rectangle('Position', [obstacle(1, i)-0.5, obstacle(2, i)-0.5, 1, 1], 'Curvature', [1,1], 'FaceColor', 'b');
% end

plot(center(1,:), center(2,:), '*g', 'MarkerSize', 1);

%%%%%%%%%%%%%%%%%% 
hold on 
point= path_plan_direct_around_version1and2_euclidean(infoGain_cluster,  begin, center, obstacle, height, width);

