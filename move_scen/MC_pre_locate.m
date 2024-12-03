clc; clear;
%% load data
% Scene 1
% filename='m1_acc.txt';               
% [ax,ay,az,time_acc] = readacc(filename);    
% 
% filename='m1_gyro.txt';
% [tx,ty,tz,time_gyro] = readacc(filename);
% 
% filename='m1_wifi.txt';
% [time_wifi,sign,RSSI] = readwifi(filename);

% Scene 2
filename='rb2_acc.txt';              
[ax,ay,az,time_acc] = readacc_v2(filename,2);  

filename='rb2_gyro.txt';
[tx,ty,tz,time_gyro] = readacc_v2(filename,2);

filename='rb2_wifi.txt';
[time_wifi,sign,RSSI] = readwifi_v2(filename,2);


%% Time data processing
time_acc = datenum(time_acc);  
time_gyro = datenum(time_gyro);
time_wifi = datenum(time_wifi);

[time_com, idx_acc, idx_wifi] = intersect(time_acc, time_wifi); 


%% Synchronize initial data with effective time
time_acc = datestr(time_acc(idx_acc(1):idx_acc(end)+4),'mm-dd HH:MM:SS');
ax = ax(idx_acc(1):idx_acc(end)+4);
ay = ay(idx_acc(1):idx_acc(end)+4);
az = az(idx_acc(1):idx_acc(end)+4);
all = ax + ay + az;     

time_gyro = datestr(time_gyro(idx_acc(1):idx_acc(end)+4),'mm-dd HH:MM:SS');
tx = tx(idx_acc(1):idx_acc(end)+4);
ty = ty(idx_acc(1):idx_acc(end)+4);
tz = tz(idx_acc(1):idx_acc(end)+4);

RSSI = RSSI(idx_wifi(1):end);
sign = sign(idx_wifi(1):end);
time_wifi = datestr(time_wifi(idx_wifi(1):end),'mm-dd HH:MM:SS');


%% divide into groups
p = 1;
for i=1:length(time_wifi)-1
    if strcmp (time_wifi(i,:), time_wifi(i+1,:)) == 0 
        pos(p) = i; 
        p = p+1;  
    end
end


%% Processing gyroscope data
% Call Gyro_data function, input gyro data tx,ty,tz
% Output: turn_num - number of turns; turn_time - turn time; turn_angle - turn angle
[turn_num, turn_time, turn_angle] = Gyro_data(tx,ty,tz);


%% Processing of linear acceleration data
% Call the Acc_data function: total acceleration all; common time period time_com; turn time turn_time
% Output: time_step - sampling time; step - sampling distance
[time_step, step] = Acc_data(all, time_com, turn_time);


%% Calculate PDR coordinates
x = zeros(length(step),1);
y = zeros(length(step),1);
z = zeros(length(step),1);

theta = 0;
j = 1;
for i = 1:1:length(time_step)-1
    if isempty(turn_time) == 1
        theta = 0;
    elseif time_step(i) == turn_time(j)
        theta = theta + turn_angle(j);
        j = j + 1;
        if  j > turn_num
            j = turn_num;
        end
        
    end
    
    x(i+1) = x(i) + step(i+1) * cosd(theta);
    y(i+1) = y(i) + step(i+1) * sind(theta);
end

% 绘制轨迹
% figure
% plot(x,y,'LineWidth',2);
% title('PDR');


%% *********  MAIN  *********
%% RSSI to distance
A = 31.5;
n = 3.15;
d = 10.^((abs(RSSI)-A)./(10*n));

cla = p; 
dr = cell(1,cla);
signr = cell(1,cla);
for i = 0:1:cla-1
    if i == 0
        dr{1} = d(1:pos(1));
        signr{1} = sign(1:pos(1));
    elseif i == cla-1
        dr{i+1} = d(pos(i)+1:end);
        signr{i+1} = sign(pos(i)+1:end);
    else
        dr{i+1} = d(pos(i)+1:pos(i+1));
        signr{i+1} = sign(pos(i)+1:pos(i+1));
    end
end


%% Modification: each segment is divided into 7 sampling points
group = 7;
gp = ceil(cla/group);
mod_p = mod(cla,group);
bq = group - mod_p;

L = zeros(gp,group);
if gp > 1
    L(1,:) = 1:1:group;
    for i = 2:gp-1
        L(i,:) = (i-1)*group+1:1:i*group;
    end
    L(gp,:) = [L(gp-1,end-bq+1:end), (gp-1)*group+1:1:(gp-1)*group+mod_p];
elseif gp == 1
    L(1,:) = 1:1:group;
end

CAP = cell(gp,group);
dap = cell(gp,group);
COM = cell(gp,1); 
r_com = cell(gp,group); 
result = cell(1,gp);
sta = cell(1,gp);
STA = cell(1,gp);
C_STA = cell(1,gp);


%% Determine track coordinates for synchronization
pos = zeros(turn_num,1);
for i = 1:1:turn_num
    pos(i) = find(time_step == turn_time(i));
end

x(pos) = [];
y(pos) = [];
z(pos) = [];

xap = cell(1,gp);
yap = cell(1,gp);
zap = cell(1,gp);
rap = cell(1,gp);

%% MC（pro）
pt = 1000; 
N = 25; 
K = 5;

for i = 1:1:gp
    xap{i} = x(L(i,:));
    yap{i} = y(L(i,:));
    zap{i} = z(L(i,:));
    rap{i} = cell(1,group);
    
    for j = 1:1:group
        CAP{i,j} = signr{L(i,j)};
        dap{i,j} = dr{L(i,j)};
    end
    
    COM{i} = CAP{i,1};
    for j = 1:1:group-1
        COM{i} = intersect(COM{i},CAP{i,j+1});
    end
    
    for j = 1:1:group
        posr = find(ismember(CAP{i,j},COM{i}));
        r_com{i,j} = dap{i,j}(posr);
        if j == 1
            rap{i} = {r_com{i,j}};
        else
            rap{i} = [rap{i},{r_com{i,j}}];
        end
    end
    
    [result{i},sta{i},STA{i},C_STA{i}] = Monte_pro(COM{i},group,pt,N,K,xap{i},yap{i},zap{i},rap{i});
end




%% Filtering the same sta - weighting
if gp > 1
    com = COM{1};
    for i = 1:1:gp
        com = intersect(com,COM{i});
    end
    
    r_sta = cell(gp,1);
    sta_com = cell(gp,1);
    dm = cell(gp,1);   
    d_err = cell(gp,1);
    wt = zeros(length(com),gp); 
    for i = 1:1:gp
        posr = find(ismember(COM{i},com));
        r_sta{i} = zeros(length(com),group);
        dm{i} = zeros(length(com),group);
        d_err{i} = zeros(length(com),group);
        for j = 1:1:group
            r_sta{i}(:,j) = r_com{i,j}(posr);
        end
        sta_com{i} = cell2mat(sta{i}(posr)');
        for j = 1:1:group
            dm{i}(:,j) = sqrt((sta_com{i}(:,1) - xap{i}(j)).^2 + ...
                (sta_com{i}(:,2) - yap{i}(j)).^2 + ...
                (sta_com{i}(:,3) - zap{i}(j)).^2);
            d_err{i}(:,j) = sqrt((dm{i}(:,j).^2 - r_sta{i}(:,j).^2).^2);
        end
        wt(:,i) = 1./sum(d_err{i},2);
    end
    
    wifi_sta = zeros(length(com),3);
    for i = 1:1:length(com)
        wifi_sta(i,:) = (sta_com{1}(i,:)*wt(i,1) + sta_com{2}(i,:)*wt(i,2) + sta_com{3}(i,:)*wt(i,3))/sum(wt(i,:));
    end
    
elseif gp == 1
    wifi_sta = cell2mat(sta{1}'); % 存储结果
    STA = STA{1};
end
