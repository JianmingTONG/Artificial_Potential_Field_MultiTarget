function potential_viewer()%, obstacle)
    
    load apf.mat
    width  = size(map, 2);
    height = size(map, 1);
    axis([0 352 0 224]);
    begin = [237;170];
    obstacle = []; 
    target = [];
    
    for i = 2 : height-1
        for j = 2 : width-1
            if(map(i,j) == 100)
                obstacle = [obstacle, [i;j]];
            end
            if(map(i,j)==255)
                temp = 0;
                temp = temp + (map(i+1, j)==0);
                temp = temp + (map(i, j+1)==0);
                temp = temp + (map(i-1, j)==0);
                temp = temp + (map(i, j-1)==0);
                if(temp>= 1)
                    target = [target, [i;j]];
                end
            end
        end
    end

%     resolution = 0.25;
% 
%     sequence = 1:resolution:14;
    z=zeros(width ,height);
    for i = 1 : height
        for j = 1 : width
            z(j,i) = computNewPotentialMultiGoal_with_map(map, [i; j], target, obstacle, height, width);
%             z(i,j) = computNewPotentialMultiGoal([1+i*resolution; 1+j*resolution], target, obstacle);
        end
    end

    h = mesh(z);   
    ylabel({'$ Y $'},'Interpreter','latex','FontSize',5);
    xlabel({'$ X $'},'Interpreter','latex','FontSize',5);

end

