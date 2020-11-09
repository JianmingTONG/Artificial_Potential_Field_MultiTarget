function MRPF_potential_viewer_version2()%, obstacle)
    clc, clear all, close all
    load apf_paper2.mat
    height = size(map,1);
    width  = size(map,2);
 
    begin =[450; 40];
    inflation_radius = 5;
    % obstacle=[0 0 0 0 0 0 0 0 0 0 0  0  0  0  0  0   15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 1  2  3  4  5  6  7  8  9  10 11 12 13 14   1 2 3 4 5 6 7 8 9 10 11 12 13 14   5 5 5 5 5 5 5 5 5  5  5  5  10 10 10 10 10 10 10 10; 
    %           0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15  0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 0 0 0 0 0 0 0 0 0  0  0  0  0  0   2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6  9  13];
    % obstacle=[1 1 1 1 1 1 1 1 1 1  1  1  1  1  1     2  3  4  5  6  7  8  9  10 11 12 13 14    2  3  4  5  6  7  8  9  10 11 12 13 14 15     15 15 15 15 15 15 15 15 15 15 15 15 15 15     5 5 5 5 5 5 5 5 5  5  5  5   10 10 10 10 10 10 10 10; 
    %           1 2 3 4 5 6 7 8 9 10 11 12 13 14 15    15 15 15 15 15 15 15 15 15 15 15 15 15    1  1  1  1  1  1  1  1  1  1  1  1  1   1     2  3  4  5  6  7  8  9  10 11 12 13 14 15     1 2 3 4 5 6 7 8 9  10 11 12  14 13 12 11 10 9  8  7];

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

    %%%%%%%%%%%%%%%%%% New Solution: Cluster Targets into centers; 
    %%%%%%%%%%%%%%%%%% InfoGain = the number of targets in each cluster.
    %%%%%%%%%%%%%%%%%% Note: the K_repu coefficient should be set as 4;
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


    %%%%%%%%%%%%%%%%%% Calculate distance maps starting from targets.
    dismap = zeros(size(center,2), height, width);
    for i = 1:size(center,2)
        dismap(i,:,:) = map_distance_generation_version1and2(map ,center(:,i), obstacle, height, width);
    end
    %%%%%%%%%%%%%%%%%% 


    %%%%%%%%%%%% view potential
    path = [];
    z=zeros(width ,height);
    parfor i = 1 : height
        for j = 1 : width
            z(j,i) = computNewPotentialMultiGoal_version1and2(infoGain_cluster, dismap, [i;j], center, obstacle, path);
        end
    end
    
 
    figure;
%     z(find( z < -100)) = 0;
%     z(find( z > 100)) = 10;
%     z(find( abs(z + 0.9367) < 0.01)) =-inf;
    plot_z = z;
    plot_z(find( plot_z < -2)) = -2;
%     z(find( z > 100)) = 10;
    zlim([-2 0]);
%     h = mesh(plot_z); 
    h = surf(plot_z,'EdgeColor', 'none');  
%     set(gcf,'position',[120 100 1000 810])
%     xlabel({'$ X $'},'Interpreter','latex','FontSize',10);
%     ylabel({'$ Y $'},'Interpreter','latex','FontSize',10);
    zlim([-3 0]);
     %%%%%%%%%%%% view the path
   %%%%%%%%%%%%%%%%%% Calculate distance maps starting from targets.
    hold on
    dismap = zeros(size(center,2), height, width);
    for i = 1:size(center,2)
        dismap(i,:,:) = map_distance_generation_version1and2(map ,center(:,i), obstacle, height, width);
    end
    %%%%%%%%%%%%%%%%%% 
    temp = computNewPotentialMultiGoal_version1and2(infoGain_cluster, dismap, begin, center, obstacle, []);
    plot3(begin(1),begin(2),temp,'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
    hold on
    point= path_plan_direct_around_version1and2_3d(infoGain_cluster, dismap, begin, center, obstacle, height, width);
    
    xlabel({'$ X/{\rm m} $'},'Interpreter','latex','FontSize',30);
    ylabel({'$ Y/{\rm m} $'},'Interpreter','latex','FontSize',30);
    zlabel({'$ Z/{\rm m} $'},'Interpreter','latex','FontSize',30);

    zlim([-3 0]);
%     legend({'$ MMPF $','$Robot\ location$','$MMPF\ selection$'},'Interpreter','latex', 'Location', 'NorthEast' );
%     legend({'MMPF','Robot Location','MMPF Selection'}, 'Location', 'NorthEast' );
     legend({'MMPF','Robot Location','MMPF   Selection'},'Interpreter','latex' , 'Location', 'NorthWest' );
    set(gca,'XTick',[0 500]);
    set(gca,'XTicklabel',{'0','500'});
    set(gca,'YTick',[-1 350]);
    set(gca,'YTicklabel',{'0','280'});
    set(gca,'ZTick',[-3 0]);
    set(gca,'ZTicklabel',{'-3','0'});
%     legend({'$ MRPF $','$MRPF\ selection\ goal\ from\ 4\ centroids$'},'Interpreter','latex', 'Location', 'NorthWest' );
    set(gca,'FontSize',30);
    set(gcf,'position',[100 100 785 500])
    savepdf(gcf,'MMPF.pdf')

 
end

