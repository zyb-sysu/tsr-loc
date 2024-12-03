%% ************** generate *******************
function [xp,yp] = re_generate(x_max,y_max,R,pt)

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

[xp(1:p1),yp(1:p1)] = Csamp(x_max(1),y_max(1),R,p1);
[xp(p1+1:p2),yp(p1+1:p2)] = Csamp(x_max(2),y_max(2),R,p1);
[xp(p2+1:p3),yp(p2+1:p3)] = Csamp(x_max(3),y_max(3),R,p1);
[xp(p3+1:p4),yp(p3+1:p4)] = Csamp(x_max(4),y_max(4),R,p1);
[xp(p4+1:p5),yp(p4+1:p5)] = Csamp(x_max(5),y_max(5),R,p1);
[xp(p5+1:p6),yp(p5+1:p6)] = Csamp(x_max(6),y_max(6),R,p1);
[xp(p6+1:p7),yp(p6+1:p7)] = Csamp(x_max(7),y_max(7),R,p1);
[xp(p7+1:p8),yp(p7+1:p8)] = Csamp(x_max(8),y_max(8),R,p1);
[xp(p8+1:p9),yp(p8+1:p9)] = Csamp(x_max(9),y_max(9),R,p1);
[xp(p9+1:p10),yp(p9+1:p10)] = Csamp(x_max(10),y_max(10),R,p1);
xp(p10+1:pt) = x_max;
yp(p10+1:pt) = y_max;

end
