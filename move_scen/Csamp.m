%% resample
function [x,y] = Csamp(x0,y0,R,num_Dian)
r = R*sqrt(rand(1,num_Dian));
angle1 = 2*pi*rand(1,num_Dian);
x = x0 + r.*cos(angle1);
y = y0 + r.*sin(angle1);
end