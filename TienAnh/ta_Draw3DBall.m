function h=ta_Draw3DBall(r,position,color)
% Modify funtion Draw3DBall by TienAnh
% Add argument choose color
hold on
[x,y,z] = sphere();
h=surf( r*x+position(1), r*y+position(2), r*z+position(3));
set(h,'FaceColor',color,'EdgeAlpha', 0);