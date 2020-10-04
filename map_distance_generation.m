function map = map_distance_generation(curr, obstacle, map_height, map_width)
    map = zeros(map_height, map_width);
    for i = 1: size(obstacle, 2)
        map(obstacle(1,i), obstacle(2,i)) = 100;
    end
    curr_grow_boundary = zeros(2,1);
    curr_grow_boundary(1, 1) = floor(curr(1));
    curr_grow_boundary(2, 1) = floor(curr(2));
    next_grow_boundary = [;];
    exhaustAllPoints = 1;
    iteration = 1;
    while(exhaustAllPoints)
        condition = 0;
        for i = 1: size(curr_grow_boundary,2)
            if  (curr_grow_boundary(1,i)+1) <= 15
                if map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i)) == 0
                    map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i))  = iteration;
                    temp = [curr_grow_boundary(1,i)+1; curr_grow_boundary(2,i)];
                    next_grow_boundary = [next_grow_boundary, temp];
                    condition = 1;
                end
            end
            
            if  ((curr_grow_boundary(1,i)+1) <= 15) &&  ((curr_grow_boundary(2,i)+1) <= 15)
                if map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i)+1) == 0
                    map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i)+1)  = iteration;
                    temp = [curr_grow_boundary(1,i)+1; curr_grow_boundary(2,i)+1];
                    next_grow_boundary = [next_grow_boundary, temp];
                    condition = 1;
                end
            end
            
            if (curr_grow_boundary(2,i)+1) <= 15
                if map(curr_grow_boundary(1,i), curr_grow_boundary(2,i)+1) == 0
                    map(curr_grow_boundary(1,i), curr_grow_boundary(2,i)+1)  = iteration;
                    temp = [curr_grow_boundary(1,i); curr_grow_boundary(2,i)+1];
                    next_grow_boundary = [next_grow_boundary, temp];
                    condition = 1;
                end
            end
            if (curr_grow_boundary(1,i)-1) > 0 
                if (curr_grow_boundary(2,i)+1) <= 15
                    if map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i)+1) == 0
                        map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i)+1)  = iteration;
                        temp = [curr_grow_boundary(1,i)-1; curr_grow_boundary(2,i)+1];
                        next_grow_boundary = [next_grow_boundary, temp];
                        condition = 1;
                    end
                end
                if map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i)) == 0
                    map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i))  = iteration;
                    temp = [curr_grow_boundary(1,i)-1; curr_grow_boundary(2,i)];
                    next_grow_boundary = [next_grow_boundary, temp];
                    condition = 1;
                end
                if  (curr_grow_boundary(2,i)-1) > 0
                     if map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i)-1) == 0
                        map(curr_grow_boundary(1,i)-1, curr_grow_boundary(2,i)-1)  = iteration;
                        temp = [curr_grow_boundary(1,i)-1; curr_grow_boundary(2,i)-1];
                        next_grow_boundary = [next_grow_boundary, temp];
                        condition = 1;
                     end
                end
            end
           
            if  (curr_grow_boundary(2,i)-1) > 0
                if map(curr_grow_boundary(1,i), curr_grow_boundary(2,i)-1) == 0
                    map(curr_grow_boundary(1,i), curr_grow_boundary(2,i)-1)  = iteration;
                    temp = [curr_grow_boundary(1,i); curr_grow_boundary(2,i)-1];
                    next_grow_boundary = [next_grow_boundary, temp];
                    condition = 1;
                end
                if  (curr_grow_boundary(1,i)+1) <= 15
                    if map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i)-1) == 0
                        map(curr_grow_boundary(1,i)+1, curr_grow_boundary(2,i)-1)  = iteration;
                        temp = [curr_grow_boundary(1,i)+1; curr_grow_boundary(2,i)-1];
                        next_grow_boundary = [next_grow_boundary, temp];
                        condition = 1;
                    end
                end
            end
        end
        
        if(condition == 0)
            exhaustAllPoints = 0;
        end
        iteration = iteration + 1;
        curr_grow_boundary = next_grow_boundary;


    end
end