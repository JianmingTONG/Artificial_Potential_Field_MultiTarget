function potential_viewer()%, obstacle)
    curr = [2; 2];
    resolution = 0.25;
    % over = [5 14; 14 14];
    % obstacle=[5 5 5 5 5 8 8 8 8 8 8 8 10 10 ;5 6 7 8 9 15 14 13 12 11 10 6 9 13];
    % obstacle=[5 5 5 5 5 5 5 5 5 5 5 5 10 10 10 10 10 10 10 10 10 ;1 2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6 9 13];
%     obstacle=[0 0 0 0 0 0 0 0 0 0 0  0  0  0  0  0  15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 1  2  3  4  5  6  7  8  9  10 11 12 13 14   1 2 3 4 5 6 7 8 9 10 11 12 13 14   5 5 5 5 5 5 5 5 5  5  5  5  10 10 10 10 10 10 10 10; 
%     0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 0 0 0 0 0 0 0 0 0  0  0  0  0  0   2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6  9  13];
    target  =[14;14]; %[13 13 13 13 0 0 0; 10 11 12 13 10 12 13];
    obstacle=[1 1 1 1 1 1 1 1 1 1  1  1  1  1  1     2  3  4  5  6  7  8  9  10 11 12 13 14  2  3  4  5  6  7  8  9  10 11 12 13 14 15     15 15 15 15 15 15 15 15 15 15 15 15 15 15     5 5 5 5 5 5 5 5 5  5  5  5   10 10 10 10 10 10 10 10; 
              1 2 3 4 5 6 7 8 9 10 11 12 13 14 15    15 15 15 15 15 15 15 15 15 15 15 15 15  1  1  1  1  1  1  1  1  1  1  1  1  1   1     2  3  4  5  6  7  8  9  10 11 12 13 14 15     1 2 3 4 5 6 7 8 9  10 11 12  14 13 12 11 10 9  8  7];
    
    sequence = 1:resolution:14;
    z=zeros(size(sequence,2),size(sequence,2));
    for i = 1 : size(sequence,2)
        for j = 1 : size(sequence,2)
        %          z(i,j) = computP([i,j], [14, 14], obstacle);
        %          z(i,j) = computMultiGoalP([i,j], target, obstacle);
            z(i,j) = computNewPotentialMultiGoal([1+i*resolution; 1+j*resolution], target, obstacle);
        end
    end

    h = mesh(z);   
    ylabel({'$ Y $'},'Interpreter','latex','FontSize',5);
    xlabel({'$ X $'},'Interpreter','latex','FontSize',5);

end

