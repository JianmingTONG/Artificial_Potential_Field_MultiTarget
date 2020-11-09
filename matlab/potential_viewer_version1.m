function potential_viewer_version1()%, obstacle)
    clc, clear all;

    load apf2.mat
    height = size(map,1);
    width  = size(map,2);

    % begin=[2;2];

    begin =[74; 115];
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
        for j = 2:size(map, 2)-1
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
    for i = 1: size(target, 2)
        temp = 0;
        for j = 1 : size(obstacle, 2)
           dis = abs(target(1, i)-obstacle(1,j)) ;
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

    %%%%%%%%%%%%%%%%%% Original infogain calculation (Calculate all targets 
    %%%%%%%%%%%%%%%%%% InfoGain is the number of unexplored neighbors around
    %%%%%%%%%%%%%%%%%% the unexplored targets. 
    %%%%%%%%%%%%%%%%%% Note: the K_repu coefficient should be set as 4;
    infoGain = zeros(1, size(targets, 2));
    for i = 1: size(targets, 2)

       tempInfoGain = 0;
       tempInfoGain = tempInfoGain + (map(targets(1,i)+1, targets(2,i)) == -1);
       tempInfoGain = tempInfoGain + (map(targets(1,i), targets(2,i)-1) == -1);
       tempInfoGain = tempInfoGain + (map(targets(1,i)-1, targets(2,i)) == -1);
       tempInfoGain = tempInfoGain + (map(targets(1,i), targets(2,i)+1) == -1);

       infoGain(i)  = tempInfoGain;

    end

    %%%%%%%%%%%%%%%%%% Calculate distance maps starting from targets.
    dismap = zeros(size(targets,2), height, width);
    parfor i = 1:size(targets,2)
        dismap(i,:,:) = map_distance_generation_version1and2(map ,targets(:,i), obstacle, height, width);
    end
    %%%%%%%%%%%%%%%%%% 

    % view path
    path = [];
    z=zeros(width ,height);
    parfor i = 1 : height
        for j = 1 : width
            z(j,i) = computNewPotentialMultiGoal_version1and2(infoGain, dismap, [i;j], target, obstacle, path);
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
 
end
