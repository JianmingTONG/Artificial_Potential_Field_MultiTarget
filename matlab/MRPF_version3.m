clc, clear all;

load apf_problem.mat
height = size(map,1);
width  = size(map,2);

% begin=[2;2];

begin =[434; 304];
% begin =[10; 180];

% over=[14 14;5 14];
% over=[14 6 14 ;14 14 2];

inflation_radius = 4;
% obstacle=[0 0 0 0 0 0 0 0 0 0 0  0  0  0  0  0   15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 1  2  3  4  5  6  7  8  9  10 11 12 13 14   1 2 3 4 5 6 7 8 9 10 11 12 13 14   5 5 5 5 5 5 5 5 5  5  5  5  10 10 10 10 10 10 10 10; 
%           0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15  0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 0 0 0 0 0 0 0 0 0  0  0  0  0  0   2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6  9  13];
% obstacle=[1 1 1 1 1 1 1 1 1 1  1  1  1  1  1     2  3  4  5  6  7  8  9  10 11 12 13 14    2  3  4  5  6  7  8  9  10 11 12 13 14 15     15 15 15 15 15 15 15 15 15 15 15 15 15 15     5 5 5 5 5 5 5 5 5  5  5  5   10 10 10 10 10 10 10 10; 
%           1 2 3 4 5 6 7 8 9 10 11 12 13 14 15    15 15 15 15 15 15 15 15 15 15 15 15 15    1  1  1  1  1  1  1  1  1  1  1  1  1   1     2  3  4  5  6  7  8  9  10 11 12 13 14 15     1 2 3 4 5 6 7 8 9  10 11 12  14 13 12 11 10 9  8  7];

obstacle = []; 
target = [];
for i = 2:size(map, 1)-1
    for j = 2:size(map, 2 )-1
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

%% Original APF -- Deprecated for performance downgradion
%%%%%%%%%%%%%%%%%% Remove obstacles whose four direct neightbors are all obstacles
obstacles = [];

for i = 1: size(obstacle,2)
    if(map(obstacle(1,i)+1, obstacle(2,i))==100 && map(obstacle(1,i)-1, obstacle(2,i))==100 && map(obstacle(1,i), obstacle(2,i)-1)==100 && map(obstacle(1,i), obstacle(2,i)+1)==100)
        map(obstacle(1,i), obstacle(2,i)) = 10000;
        continue;
    end
    
    obstacles = [obstacles, obstacle(:,i)];
end
%%%%%%%%%%%%%%%%%% 


%%%%%%%%%%%%%%%%%% Original infogain calculation (Calculate all targets 
%%%%%%%%%%%%%%%%%% InfoGain is the number of unexplored neighbors around
%%%%%%%%%%%%%%%%%% the unexplored targets. 
%%%%%%%%%%%%%%%%%% Note: the K_repu coefficient should be set as 4;
infoGain = zeros(1, size(targets, 2));
for i = 1: size(targets, 2)

   tempInfoGain = 0;
   tempInfoGain = tempInfoGain + (map(targets(1,i)+1, targets(2,i)) ==-1);
   tempInfoGain = tempInfoGain + (map(targets(1,i), targets(2,i)-1) ==-1);
   tempInfoGain = tempInfoGain + (map(targets(1,i)-1, targets(2,i)) ==-1);
   tempInfoGain = tempInfoGain + (map(targets(1,i), targets(2,i)+1) ==-1);
   
   infoGain(i) = tempInfoGain;
   
end


%%%%%%%%%%%%%%%%%% Display targets and obstacles
figure(1);
plot(begin(1), begin(2),'*b','MarkerSize', 1);
axis([0 height 0 width]);
hold on;
plot(targets(1,:), targets(2,:), '*r', 'MarkerSize', 1);

for i=1:size(obstacle, 2)
    rectangle('Position', [obstacle(1, i)-0.5, obstacle(2, i)-0.5, 1, 1], 'Curvature', [1,1], 'FaceColor', 'r');
end
%%%%%%%%%%%%%%%%%% 

point= path_plan_direct_around_version3(infoGain, map, begin, targets, obstacle, height, width);


%% New APF method to increase its performance using cluster
% 
% 
% %%%%%%%% New Version4
% targets_process = targets;
% center = [];
% infoGain_cluster = [];
% while(size(targets_process, 2) > 0)
%     idx = 1;
%     target_cluster = targets_process(:, end);
%     targets_process(:,end) = [];
%     condition = 1;
%     while(condition)
%         condition = 0;
%         for idx =  size(targets_process,2): -1:1
%             for i = 1: size(target_cluster,2)
%                if(abs(targets_process(1, idx) - target_cluster(1, i)) + abs(targets_process(2, idx) - target_cluster(2, i)) < 3)
%                    target_cluster = [target_cluster, targets_process(:, idx)];
%                    targets_process(:, idx) = [];
%                    condition= 1;
%                    break;
%                end
%             end
%         end
%     end
%     center_temp = zeros(2,1);
%     num_ = size(target_cluster,2);
%     for j = 1: num_
%        center_temp(1) =  center_temp(1) +  target_cluster(1,j);
%        center_temp(2) =  center_temp(2) +  target_cluster(2,j);
%     end
%     
%     center_temp(1) = center_temp(1) / num_;
%     center_temp(2) = center_temp(2) / num_;
%     
%     min_dis = 100;
%     min_idx = 0;
%     
%     for j = 1: num_
%         dis = abs(center_temp(1) - target_cluster(1,j)) +  abs(center_temp(2) - target_cluster(2,j));
%         if(dis < min_dis)
%            min_dis = dis;
%            min_idx= j;
%         end
%     end
%     center = [center, target_cluster(:,min_idx)];  
%     infoGain_cluster =[infoGain_cluster ,num_ ];
% end
% %%%%%%%%%%%%%%%%%% Version 4 ends
% 
% 
% 
% %%%%%%%%%%%%%%%%%% Display targets and obstacles
% figure(1);
% plot(begin(1), begin(2),'*b','MarkerSize', 1);
% axis([0 height 0 width]);
% hold on;
% plot(targets(1,:), targets(2,:), '*r', 'MarkerSize', 1);
% 
% plot(obstacle(1,:), obstacle(2,:), '*k', 'MarkerSize', 1);
% 
% % for i=1:size(obstacle, 2)
% %     rectangle('Position', [obstacle(1, i)-0.5, obstacle(2, i)-0.5, 1, 1], 'Curvature', [1,1], 'FaceColor', 'b');
% % end
% 
% plot(center(1,:), center(2,:), '*g', 'MarkerSize', 1);
% 
% %%%%%%%%%%%%%%%%%% 
% 
% 
% %%%%%%%%%%%%%%%%%% Calculate distance maps starting from targets.
% dismap = zeros(size(center,2), height, width);
% parfor i = 1:size(center,2)
%     dismap(i,:,:) = map_distance_generation_version1and2(map ,center(:,i), obstacle, height, width);
% end
% %%%%%%%%%%%%%%%%%% 