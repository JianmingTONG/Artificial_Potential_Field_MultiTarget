clc, clear all;

load map.mat
height = size(map,1);
width  = size(map,2);

begin =[450; 40];

inflation_radius = 5;
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

%%%%%%%%%%%%%%%%%% New Solution: Cluster Targets into centers based on the
%%%%%%%%%%%%%%%%%% geometry feature of frontiers that all frontiers are
%%%%%%%%%%%%%%%%%% located in the continuous line or curve
%%%%%%%%%%%%%%%%%% InfoGain = the number of targets in each cluster.
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
%%%%%%%%%%%%%%%%%% 



%%%%%%%%%%%%%%%%%% Display targets and obstacles
figure(1);
plot(begin(1), begin(2),'*b','MarkerSize', 1);
axis([0 height 0 width]);
hold on;
plot(targets(1,:), targets(2,:), '*r', 'MarkerSize', 1);

plot(obstacle(1,:), obstacle(2,:), '*k', 'MarkerSize', 1);

plot(center(1,:), center(2,:), '*g', 'MarkerSize', 1);

%%%%%%%%%%%%%%%%%% 

%% Using MMPF
%%%%%%%%%%%%%%%%% Calculate distance maps starting from targets.
dismap = zeros(size(center,2), height, width);
parfor i = 1:size(center,2)
    dismap(i,:,:) = map_distance_generation(map ,center(:,i), obstacle, height, width);
end
%%%%%%%%%%%%%%%%%% 
hold on 
point= path_plan(infoGain_cluster, dismap, begin, center, obstacle, height, width);



