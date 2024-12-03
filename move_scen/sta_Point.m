%% ************** Point *******************
function [x,y,z] = sta_Point(x0,y0,z0,R,num_Dian)
r = R*sqrt(rand(1,num_Dian));
angle1 = 2*pi*rand(1,num_Dian);
angle2 = acos(rand(1,num_Dian)*2-1);
x = x0 + r.*cos(angle1).*sin(angle2);
y = y0 + r.*sin(angle1).*sin(angle2);
z = z0 + r.*cos(angle2);
end