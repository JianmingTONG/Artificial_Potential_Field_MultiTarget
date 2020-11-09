%输出与图片尺寸相同的pdf
 %使用注意：当前不能打开过多窗口 
function savepdf(gcf,namestring)


    %gcf 为图形的句柄，namestring为想保存的文件名称，包含文件格式后缀、
    %  fp为在源文件中得到的图形位置向量 fp=get(gcf,'position'); 依次为
    %  左边缘，下边缘，图形的宽（左右长度），图形的高（上下长度）
    %  设置图片单位一定要在获得图片坐标位置之前，否则图片过大
    set(gcf,'Units','centimeters');
    fp=get(gcf,'Position');
    set(gcf,'Paperposition',[0 0 fp(3)-fp(1) fp(4)-fp(2)],'Papersize',[fp(3)-fp(1) fp(4)-fp(2)]);
    saveas(gcf,namestring)  

%%   可使用该命令使输入文件名称namestring不需要加后缀
%string=[namestring,'.pdf'];

