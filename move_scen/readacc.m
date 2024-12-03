function [ax,ay,az,time] = readacc(filename)
delimiterIn = ' ';                      
A = importdata(filename,delimiterIn);
La = length(A);
time = char(32*ones(La,14));
ax = zeros(La,1);
ay = zeros(La,1);
az = zeros(La,1);

for i = 1:La
    a = A{i};
    tp0 = find(a=='【');
    tp1 = find(a==']');
    xp = find(a=='X');
    yp = find(a=='Y');
    zp = find(a=='Z');
    
    time(i,:) = a(tp0+1:tp1-1);
    ax(i) = str2double(a(xp+2:yp-2));
    ay(i) = str2double(a(yp+2:zp-2));
    az(i) = str2double(a(zp+2:end));
end
end