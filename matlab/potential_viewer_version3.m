function potential_viewer_version3()%, obstacle)
    clc, clear all;
    load apf2.mat
    width  = size(map, 2);
    height = size(map, 1);
    
    begin =[75; 115];
    inflation_radius = 4;
    
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
                    temp1 = temp1+ (map(i+2, j)==0);
                    temp1 = temp1+ (map(i+1, j+1)==0);
                    temp1 = temp1+ (map(i+1, j-1)==0);
                    temp = temp +(temp1>0);
                end

                if(map(i, j+1)==0)
                    temp1 = 0;
                    temp1 = temp1+ (map(i, j+2)==0);
                    temp1 = temp1+ (map(i+1, j+1)==0);
                    temp1 = temp1+ (map(i-1, j+1)==0);
                    temp = temp +(temp1>0);
                end

                if(map(i-1, j)==0)
                    temp1 = 0;
                    temp1 = temp1+ (map(i-1, j+1)==0);
                    temp1 = temp1+ (map(i-1, j-1)==0);
                    temp1 = temp1+ (map(i-2, j)==0);
                    temp = temp +(temp1>0);
                end

                if(map(i, j-1)==0)
                    temp1 = 0;
                    temp1 = temp1+ (map(i, j-2)==0);
                    temp1 = temp1+ (map(i+1, j-1)==0);
                    temp1 = temp1+ (map(i-1, j-1)==0);
                    temp = temp +(temp1>0);
                end
                if(temp > 0)
                    target = [target, [i;j]];
                end
            end
        end
    end

    infoGain = zeros(1, size(target, 2));
    
    for i = 1: size(target, 2)
       tempInfoGain = 0;
       tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)) ==-1);
       tempInfoGain = tempInfoGain + (map(target(1,i), target(2,i)-1) ==-1);
       tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)) ==-1);
       tempInfoGain = tempInfoGain + (map(target(1,i), target(2,i)+1) ==-1);

       infoGain(i) = tempInfoGain;

    end

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

%     obstacles = [];
%     for i = 1: size(obstacle,2)
%         if(map(obstacle(1,i)+1, obstacle(2,i))==100 && map(obstacle(1,i)-1, obstacle(2,i))==100 && map(obstacle(1,i), obstacle(2,i)-1)==100 && map(obstacle(1,i), obstacle(2,i)+1)==100)
%             continue;
%         end
% 
%         obstacles = [obstacles, obstacle(:,i)];
%     end
    

%% VERSION3
% view distance map
    dismap = map_distance_generation_version3(map ,begin, targets, obstacle, height, width);

    h = mesh(dismap');   
    set(gcf,'position',[120 100 1000 810])
    xlabel({'$ X $'},'Interpreter','latex','FontSize',10);
    ylabel({'$ Y $'},'Interpreter','latex','FontSize',10);
    
% view path
    path = [];
    z=zeros(width ,height);
    parfor i = 1 : height
        for j = 1 : width
            z(j,i) = computNewPotentialMultiGoal_version3(infoGain, map, [i;j], targets, obstacle, height, width, path);
        end
    end
    
    figure(2);
    z(find( z < -100)) = 0;
    z(find( z > 100)) = 10;
    z(find( abs(z + 0.9367) < 0.01)) =-inf;
    h = mesh(z);  
    set(gcf,'position',[120 100 1000 810])
    xlabel({'$ X $'},'Interpreter','latex','FontSize',10);
    ylabel({'$ Y $'},'Interpreter','latex','FontSize',10);
    
%% VERSION3: use euclidean distance to measure the distance

    path = [];
    z_elu=zeros(width ,height);
    parfor i = 1 : height
        for j = 1 : width
            z_elu(j,i) = computNewPotentialMultiGoal_elu(infoGain, map, [i;j], targets, obstacle, height, width, path);
        end
    end
    
    figure(3);
    h = mesh(z_elu);  
    set(gcf,'position',[120 100 1000 810])
    xlabel({'$ X $'},'Interpreter','latex','FontSize',10);
    ylabel({'$ Y $'},'Interpreter','latex','FontSize',10);
 
end

