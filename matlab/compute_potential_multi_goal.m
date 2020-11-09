function output = compute_potential_multi_goal(infoGain, dismap, curr, target, obstacle, path)
%COMPUTP 此处显示有关此函数的摘要
%   此处显示详细说明
% k_att=10;
repu=0;
k_rep=1;
attr= 0;
Q_star=4;

%计算当前点距离终点的引力
for i=1:size(target, 2)
    dis_target= dismap(i, curr(1), curr(2));
    if(dis_target < 0.001)
       fprintf("%d ",i);
       continue
    end
    attr = attr - infoGain(i) * 1 / dis_target;
end
fprintf("\n");

%曾经走过的点由于势能积累势能增加。
for i = 1: size(path, 2)
    if(curr(1) == path(1,i) && curr(2) == path(2,i))
        attr = attr + 5;
    end
end

%计算障碍点与当前点的斥力
%设定障碍的斥力作用半径为2
% for i=1:size(obstacle, 2)
%     
%    dis_obstacle  = abs(curr(2) - obstacle(2,i)) + abs(curr(1) - obstacle(1,i));
%    if dis_obstacle <= Q_star
%         repu = repu + 1/2 * k_rep * (1/dis_obstacle - 1/Q_star)^2;
%     else
%         repu=repu+0;
%     end
% end

output = attr + repu;

end

