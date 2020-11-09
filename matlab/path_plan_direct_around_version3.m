function [ point ] = path_plan_direct_around_version3(infoGain, map, begin, target, obstacle, height, width)
    iters=1; %迭代次数
    curr=begin;
    dis = 10;
    goalPotential = 100;
    path = [];

    while ( (dis > 1) &&  (iters<=2000))
        point(:, iters)=curr;
 
        %计算当前点附近半径为0.2的8个点的势能，然后让当前点的势能减去8个点的势能取差值最大的，确定这个
        %方向，就是下一步迭代的点

        %先求这八个点的坐标
        number  = 4;
        testPoint = zeros(2, number);
        testPoint(1,1) = curr(1);
        testPoint(2,1) = curr(2)+1;
        
        testPoint(1,2) = curr(1)-1;
        testPoint(2,2) = curr(2);

        testPoint(1,3) = curr(1);
        testPoint(2,3) = curr(2)-1;
        
        testPoint(1,4) = curr(1)+1;
        testPoint(2,4) = curr(2);
        
        testOut = zeros(1,number);
        for i=1:number
            if (testPoint(1,i) < 1) || (testPoint(2,i) < 1) || (testPoint(1,i) > height) || (testPoint(2,i) > width)
                testOut(i) = 500;
            else
%                 [testOut(i), ever_been_times] = computNewPotentialMultiGoal_with_map(infoGain, map, testPoint(:,i), target, obstacle, height, width, path, ever_been_times);
                testOut(i) = computNewPotentialMultiGoal_version3(infoGain, map, testPoint(:,i), target, obstacle, height, width, path);
            end
        end
        
        [temp num] = min(testOut);
        curr=testPoint(:,num);
        plot(curr(1),curr(2),'og');
        fprintf("potential %3.2f\n", temp);
%         if (goalPotential > temp)
%             fprintf("potential %3.2f\n", temp);
%             curr=testPoint(:,num);
%             plot(curr(1),curr(2),'og');
%         else
%             curr = testPoint(:, floor(temp));
%             curr(1) = curr(1) + floor(rand()*4 - 2);
%             curr(2) = curr(2) + floor(rand()*4 - 2);
%             fprintf("potential %3.2f\n", curr);
%             plot(curr(1), curr(2), 'og');
%         end
           
        path = [path, curr];
        
% using euclidean distance to measure whether path reaches the
% one of the target frontiers.
%         for i = 1 : size(over, 2)
%             tempDis = norm(curr-over(:,i));
%             if dis > tempDis
%                 dis = tempDis;
%             end
%         end

        dis_map = map_distance_generation_version3(map, curr, target, obstacle, height, width);
        
        
        % reset invalid targets' distance value
        for i = 1:size(target,2)
           if((dis_map(target(1,i), target(2,i)) == 0) &&  ( abs(curr(1)-target(1,i)) + abs(curr(2)-target(2,i)) > 1 ) )
               dis_map(target(1,i), target(2,i)) = 1000;
           end
        end
        
        for i = 1 : size(target, 2)
            tempDis = dis_map(target(1,i), target(2,i));
            if dis > tempDis
                dis = tempDis;
            end
        end

        pause(0.01);
        iters=iters+1;
    end
end
