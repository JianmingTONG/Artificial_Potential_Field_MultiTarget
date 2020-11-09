function [ point ] = path_plan(infoGain, dismap, curr, target, obstacle, height, width)
    iters=1; %迭代次数
    dis = 10;
    path = [];
%     ever_been_times = 0;
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
                testOut(i) = compute_potential_multi_goal(infoGain, dismap, testPoint(:,i), target, obstacle, path);
            end
        end
        
        [temp num] = min(testOut);
        curr=testPoint(:,num);
        plot(curr(1),curr(2),'og');
        fprintf("%d potential %3.2f\n",iters, temp);
        path = [path, curr];
        
        for i = 1 : size(target, 2)
            tempDis = abs(curr(1)-target(1,i)) + abs(curr(2)-target(2,i));
            if dis > tempDis
                dis = tempDis;
            end
        end

        pause(0.01);
        iters=iters+1;
    end
end
