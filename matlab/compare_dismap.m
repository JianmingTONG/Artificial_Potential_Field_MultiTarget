for i = 1: size(dismap,1)
    for j = 1: size(dismap, 2)
        if(dismap(i,j) ~= dismap_c(i,j) )
            fprintf("loc:(%d, %d)",i,j);
        end
    end
end