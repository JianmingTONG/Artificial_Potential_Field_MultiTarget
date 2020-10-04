function [ point ] = path_plan(begin,over,obstacle)
    %PATH_PLAN 此处显示有关此函数的摘要
    %   此处显示详细说明
    iters=1; %迭代次数
    curr=begin;
    testR=0.2;   %测试8点的圆的半径为0.5
    dis = 10;
    while ( (dis > 0.2) &&  (iters<=5000))
        point(:, iters)=curr;
    %     attr=attractive(curr,over);
    %     repu=repulsion(curr,obstacle);
        %curoutput=computP(curr,over,obstacle);
        %计算当前点附近半径为0.2的8个点的势能，然后让当前点的势能减去8个点的势能取差值最大的，确定这个
        %方向，就是下一步迭代的点

        %先求这八个点的坐标
        testPoint = zeros(2, 8);
        testPoint(1,1) = curr(1)+1;
        testPoint(2,1) = curr(2)+1;
        
        testPoint(1,2) = curr(1);
        testPoint(2,2) = curr(2)+1;
        
        testPoint(1,3) = curr(1)-1;
        testPoint(2,3) = curr(2)+1;
        
        testPoint(1,4) = curr(1)-1;
        testPoint(2,4) = curr(2);
        
        testPoint(1,5) = curr(1)-1;
        testPoint(2,5) = curr(2)-1;
        
        testPoint(1,6) = curr(1);
        testPoint(2,6) = curr(2)-1;
        
        testPoint(1,7) = curr(1)+1;
        testPoint(2,7) = curr(2)-1;
        
        testPoint(1,8) = curr(1)+1;
        testPoint(2,8) = curr(2);
        
        testOut = zeros(1,8);
        for i=1:8
            if (testPoint(1,i) <= 0) || (testPoint(2,i) <= 0) || (testPoint(1,i) > 15) || (testPoint(2,i) > 15)
                testOut(i) = 500;
            else
                testOut(i)   = computNewPotentialMultiGoal(testPoint(:,i), over, obstacle);
            end
            %找出来最小的就可以了
        end
        [temp num]=min(testOut);

        %迭代的距离为0.1
        curr=testPoint(:,num);
        plot(curr(1),curr(2),'og');

        for i = 1 : size(over, 2)
            tempDis = norm(curr-over(:,i));
            if dis > tempDis
                dis = tempDis;
            end
        end

        pause(0.01);
        iters=iters+1;
    end
end

