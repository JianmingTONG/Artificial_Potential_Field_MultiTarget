clc, clear all;

load apf.mat
height = size(map,1);
width  = size(map,2);

axis([0 352 0 224]);
% begin=[2;2];
begin = [240;130];
curr = begin;
% over=[14 14;5 14];


% obstacle=[0 0 0 0 0 0 0 0 0 0 0  0  0  0  0  0   15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 1  2  3  4  5  6  7  8  9  10 11 12 13 14   1 2 3 4 5 6 7 8 9 10 11 12 13 14   5 5 5 5 5 5 5 5 5  5  5  5  10 10 10 10 10 10 10 10; 
%           0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15  0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 15 15 15 15 15 15 15 15 15 15 15 15 15 15 0 0 0 0 0 0 0 0 0  0  0  0  0  0   2 3 4 5 6 7 8 9 10 11 12 15 14 13 12 11 10 6  9  13];
% obstacle=[1 1 1 1 1 1 1 1 1 1  1  1  1  1  1     2  3  4  5  6  7  8  9  10 11 12 13 14    2  3  4  5  6  7  8  9  10 11 12 13 14 15     15 15 15 15 15 15 15 15 15 15 15 15 15 15     5 5 5 5 5 5 5 5 5  5  5  5   10 10 10 10 10 10 10 10; 
%           1 2 3 4 5 6 7 8 9 10 11 12 13 14 15    15 15 15 15 15 15 15 15 15 15 15 15 15    1  1  1  1  1  1  1  1  1  1  1  1  1   1     2  3  4  5  6  7  8  9  10 11 12 13 14 15     1 2 3 4 5 6 7 8 9  10 11 12  14 13 12 11 10 9  8  7];

obstacle = []; 
target = [];
for i = 2:size(map, 1)-1
    for j = 2:size(map, 2 )-1
        if(map(i,j) == 100)
            obstacle = [obstacle, [i;j]];
        end
        if(map(i,j)==255)
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

infoGain = zeros(1,size(target,2));
for i = 1: size(target,2)
   tempInfoGain = 0;
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)  ) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)  ) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)-1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)-1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)  , target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)  , target(2,i)-1) ==255);
   infoGain(i)  = tempInfoGain;
end

dis_map = map_distance_generation_with_map(map, curr, target, obstacle, height, width);
potential_map = zeros(height, width);
parfor i = 1:height
    for j = 1:width
        if(dis_map(i,j) ~= 100000)
            potential_map(i,j) = computNewPotentialMultiGoal_with_map(infoGain, map, [i,j], target, obstacle, height, width);
        end
    end
end
