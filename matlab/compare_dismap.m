temp = zeros(256,224);
for i = 1: 256%ize(dismap,1)
    for j = 1:224 %size(dismap, 2)
%         if(dismap(i,j) ~= dismap_c(i,j) )
%             fprintf("loc:(%d, %d)",i,j);
%         end
        temp(i,j) = dismap(4, i,j);
    end
end