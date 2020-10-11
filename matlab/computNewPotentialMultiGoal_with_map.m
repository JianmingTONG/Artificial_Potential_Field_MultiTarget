function [ output ] = computNewPotentialMultiGoal_with_map(infoGain, map, curr, target, obstacle, height, width)
%COMPUTP 此处显示有关此函数的摘要
%   此处显示详细说明
% k_att=10;
repu=0;
k_rep=1;
attr= 0;
Q_star=5;
dis_map = map_distance_generation_with_map(map, curr, target, obstacle, height, width);

%计算当前点距离终点的引力
target_int = zeros(2,1);
dis = zeros(1,4);


for i=1:size(target, 2)
    dis_target= dis_map(target(1,i), target(2,i));
    attr = attr - infoGain(i) * 1 / dis_target;
end

%计算障碍点与当前点的斥力
%设定障碍的斥力作用半径为2
for i=1:size(obstacle, 2)
   if dis_map(obstacle(1,i), obstacle(2,i))<=Q_star
%         repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2;
        repu=repu+1/2*k_rep*(1/dis_map(obstacle(1,i), obstacle(2,i))-1/Q_star)^2;
    else
        repu=repu+0;
    end
end

output = attr + repu;

end

