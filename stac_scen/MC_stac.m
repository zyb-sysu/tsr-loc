clc;clear; close all;

%% load data
L1 = 0:6:36;
L2 = L1;

% Pre-allocated memory ---- 10 for sampling points from 4-14, 8 for 8 targets
nf = 8;
result = cell(1,nf);
monte_err = cell(1,1);
per_all_mid = cell(1,nf);
d_err_mid = cell(1,nf);

monte_err_re = cell(1,1);
d_err_re = cell(1,nf);

num = length(L1)+length(L2);
filename = strings(num,1);
x = zeros(num,1);
y = zeros(num,1);
z = zeros(num,1);

x(1:length(L1)) = L1';
y(1:length(L1)) = 0;
z(1:length(L1)) = 0;

x(length(L1)+1:num) = L2';
y(length(L1)+1:num) = 10;
z(length(L1)+1:num) = 0;

% write filename
for i = 1:num
    filename(i) = [num2str(x(i)), '_', num2str(y(i)), '_', num2str(z(i)), '.txt'];
end

RSSI_all = cell(1,num);
C_all = cell(1,num);
mean_d = cell(1,num);
mid_d = cell(1,num);

%% RSSI to distance
A = 31.5;    % A is the absolute value of rssi at 1m from the detection device
n = 3.15;    % n is the environmental attenuation factor


for i = 1:num
    [C, RSSI_filted] = RSSI_GF(filename(i));
    RSSI_all{i} = RSSI_filted;
    mean_RSSI = mean(RSSI_filted,2); % Average filtered RSSI
    mid = ceil(size(RSSI_filted,2));
    mid_RSSI = RSSI_filted(:,mid);
    C_all{i} = C;
    mean_d{i} = 10.^((abs(mean_RSSI)-A)./(10*n));    % RSSI-distance model
    mid_d{i} = 10.^((abs(mid_RSSI)-A)./(10*n));
end


%% Fetch the detected common signal
cla = length(x);
C = C_all{1};

for i = 1:1:cla-1
    C = intersect(C,C_all{i+1});
end


%% signal grouping
posr = cell(1,cla); 
r = cell(1,cla); 
for i = 1:1:cla
    posr{i} = find(ismember(C_all{i},C));
    r{i} = mid_d{i}(posr{i});
end

% Database features-Cs(common signal)
Cs = C;

%% MC（pro）
pt = 500; % Number of randomized points
N = 20; % Number of single simulations
K = 100; % Number of consecutive simulations

%     figure;
[mean_result,sta] = Monte_pro(C,cla,pt,N,K,x,y,z,r);

wifi_sta = [];
for i = 1:1:length(C)
    wifi_sta = [wifi_sta; sta{i}];
end


%% reverse positioning
% 15_0_0; 15_3_0; 15_7_0; 15_10_0; 21_0_0; 21_3_0; 21_7_0; 21_10_0
xp(1:4) = 12;   xp(5:8) = 24;
yp(1:4) = [0,3,7,10];   yp(5:8) = [0,3,7,10];
zp(1:8) = 0;

%     xp = 18; yp = 10; zp = 0;
%     xp(1:2) = 12;   xp(3:4) = 24;
%     yp(1:2) = [0,10];   yp(3:4) = [0,10];
%     zp(1:4) = 0;

filename = strings(nf,1);
% write filename
for i = 1:nf
    filename(i) = [num2str(xp(i)), '_', num2str(yp(i)), '_', num2str(zp(i)), '.txt'];
end

C_all = cell(1,length(filename));
d_all = cell(1,length(filename));

for i = 1:length(filename)
    [C, RSSI_filted] = RSSI_GF(filename{i});
    C_all{i} = C;
    d_all{i} = 10.^((abs(RSSI_filted)-A)./(10*n));    % RSSI to distance
end
C_all_mid = C_all;
d_all_mid = d_all;

st = 4;
monte_err{st-3} = zeros(1,length(filename));
monte_err_re{st-3} = zeros(1,length(filename));

for j = 1:1:length(filename)
    p = size(d_all_mid{j},2);  % mid point
    per = zeros(p,3);
    per_STA = zeros(p,3);
    for i = 1:1:p
        [result{st-3,j},per(i,:)] = Monte_UE_pro(wifi_sta, Cs, C_all_mid{j}, d_all_mid{j}(:,i), pt, N, K);
    end
    per_all_mid{st-3,j} = per;

    %% Calculating Errors
    d_err_mid{st-3,j} = sqrt((per_all_mid{st-3,j}(:,1) - xp(j)).^2 + (per_all_mid{st-3,j}(:,2) - yp(j)).^2);
    monte_err{st-3}(j) = mean(d_err_mid{st-3,j});

    err_re = d_err_mid{st-3,j};
    re_pos = find(err_re>5);
    err_re(re_pos) = [];
    d_err_re{st-3,j} = err_re;
    monte_err_re{st-3}(j) = mean(d_err_re{st-3,j});
end



%% Error
ERR = [];
for i = 1:8
    ERR = [ERR; d_err_mid{i}];
end

figure(1);
bar(ERR);
title("MC Locate Error");

figure(2);
h = cdfplot(ERR);
set(h,'Color','b','LineWidth',2.5);
grid on;
title("MC Locate Error CDF");