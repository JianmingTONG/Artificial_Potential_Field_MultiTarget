function [ output ] = computNewPotentialMultiGoal( curr, target, obstacle, height, width)
%COMPUTP 此处显示有关此函数的摘要
%   此处显示详细说明
k_att=20;
repu=0;
k_rep=1;
attr= 0;
Q_star=2;
dis_map = map_distance_generation(curr, obstacle, height, width);

%计算当前点距离终点的引力
target_int = zeros(2,1);
dis = zeros(1,4);
for i=1:size(target, 2)
    if (target(1,i) == fix(target(1,i))) && (target(2,i) == fix(target(2,i)))
        dis_target= dis_map(target(1,i), target(2,i));
    else
        target_int(1,1) = floor(target(1,i));
        target_int(2,1) = floor(target(2,i));

        dis(1) = dis_map(target_int(1,1)  , target_int(2,1));
        dis(2) = dis_map(target_int(1,1)+1, target_int(2,1));
        dis(3) = dis_map(target_int(1,1)  , target_int(2,1)+1);
        dis(4) = dis_map(target_int(1,1)+1, target_int(2,1)+1);

        dis_x = target(1,i) - target_int(1,1);
        dis_y = target(2,i) - target_int(2,1);
        dis_target = 1/2 * ( (dis_x + dis_y) * dis(1) + (1 - dis_x + dis_y) * dis(2) + (dis_x + 1 - dis_y) * dis(3) + (2 - dis_x - dis_y) * dis(4));
    end

    attr = attr - k_att * 1 / dis_target;
end


%计算障碍点与当前点的斥力
%设定障碍的斥力作用半径为2
for i=1:size(obstacle, 2)
   if norm(curr-obstacle(:,i))<=Q_star
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2;
    else
        repu=repu+0;
    end
end

output = attr + repu;

end

