%% ************** generate *******************
function [xp,yp,zp] = sta_generate(xp,yp,zp,err,j,err_min,pt)
pos = zeros(10,1);
xp_min = zeros(1,10);
yp_min = zeros(1,10);
zp_min = zeros(1,10);

p1 = 0.1*pt - 1;
p2 = 0.2*pt - 2;
p3 = 0.3*pt - 3;
p4 = 0.4*pt - 4;
p5 = 0.5*pt - 5;
p6 = 0.6*pt - 6;
p7 = 0.7*pt - 7;
p8 = 0.8*pt - 8;
p9 = 0.9*pt - 9;
p10 = pt - 10;


for i = 1:10
    pos(i) = find(err == err_min(j,i));
    xp_min(i) = xp(pos(i));
    yp_min(i) = yp(pos(i));
    zp_min(i) = zp(pos(i));
end

[xp(1:p1),yp(1:p1),zp(1:p1)] = sta_Point(xp_min(1),yp_min(1),zp_min(1),30,p1);
[xp(p1+1:p2),yp(p1+1:p2),zp(p1+1:p2)] = sta_Point(xp_min(2),yp_min(2),zp_min(2),30,p1);
[xp(p2+1:p3),yp(p2+1:p3),zp(p2+1:p3)] = sta_Point(xp_min(3),yp_min(3),zp_min(3),30,p1);
[xp(p3+1:p4),yp(p3+1:p4),zp(p3+1:p4)] = sta_Point(xp_min(4),yp_min(4),zp_min(4),30,p1);
[xp(p4+1:p5),yp(p4+1:p5),zp(p4+1:p5)] = sta_Point(xp_min(5),yp_min(5),zp_min(5),30,p1);
[xp(p5+1:p6),yp(p5+1:p6),zp(p5+1:p6)] = sta_Point(xp_min(6),yp_min(6),zp_min(6),30,p1);
[xp(p6+1:p7),yp(p6+1:p7),zp(p6+1:p7)] = sta_Point(xp_min(7),yp_min(7),zp_min(7),30,p1);
[xp(p7+1:p8),yp(p7+1:p8),zp(p7+1:p8)] = sta_Point(xp_min(8),yp_min(8),zp_min(8),30,p1);
[xp(p8+1:p9),yp(p8+1:p9),zp(p8+1:p9)] = sta_Point(xp_min(9),yp_min(9),zp_min(9),30,p1);
[xp(p9+1:p10),yp(p9+1:p10),zp(p9+1:p10)] = sta_Point(xp_min(10),yp_min(10),zp_min(10),30,p1);
xp(p10+1:pt) = xp_min;
yp(p10+1:pt) = yp_min;
zp(p10+1:pt) = zp_min;
end
