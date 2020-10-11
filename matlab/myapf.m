clc, clear all;
figure(1);
load apf.mat
height = size(map,1);
width  = size(map,2);

axis([0 352 0 224]);
% begin=[2;2];
begin = [240;130];
% over=[14 14;5 14];
% over=[14 6 14 ;14 14 2];
hold on;

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
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)+1, target(2,i)-1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i)-1, target(2,i)-1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i), target(2,i)+1) ==255);
   tempInfoGain = tempInfoGain + (map(target(1,i), target(2,i)-1) ==255);
   infoGain(i) = tempInfoGain;
end

plot(begin(1),begin(2),'*b','MarkerSize',1);
plot(target(1,:),target(2,:),'*r','MarkerSize',1);

for i=1:size(obstacle,2)
    rectangle('Position',[obstacle(1,i)-0.5, obstacle(2,i)-0.5, 1, 1],'Curvature',[1,1],'FaceColor','r');
end

point= path_plan_direct_around(infoGain, map, begin, target, obstacle, height, width);
