%% Position estimation
%% load data
% Scene 1
% filename = 'm3_wifi.txt';
% [time_wifi,sign,RSSI] = readwifi(filename);

% Scene 2
filename = 'rb3_wifi.txt';
[time_wifi,sign,RSSI] = readwifi_v2(filename,2);

%% ACC data
filename='rb3_acc.txt';
[ax,ay,az,time_acc] = readacc_v2(filename,2);

%% time synchronization
time_acc = datenum(time_acc);
time_wifi = datenum(time_wifi);
[time_com, idx_acc, idx_wifi] = intersect(time_acc, time_wifi);

time_acc = datestr(time_acc(idx_acc(1):idx_acc(end)+4),'mm-dd HH:MM:SS');
ax = ax(idx_acc(1):idx_acc(end)+4);
ay = ay(idx_acc(1):idx_acc(end)+4);
az = az(idx_acc(1):idx_acc(end)+4);
all = ax + ay + az;

RSSI = RSSI(idx_wifi(1):end);
sign = sign(idx_wifi(1):end);
time_wifi = datestr(time_wifi(idx_wifi(1):end),'mm-dd HH:MM:SS');

%% output step
[~, step] = Acc_data(all, time_com, []);

p = 1;
pos = [];
for i=1:length(time_wifi)-1
    if strcmp (time_wifi(i,:), time_wifi(i+1,:)) == 0
        pos(p) = i;
        p = p+1;
    end
end


%% RSSI to distance
A = 31.5;
n = 3.15;
d = 10.^((abs(RSSI)-A)./(10*n));


%% grouping
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


%% MAIN
if p > 1
    per = zeros(p,3);
    for i = 1:1:p
        if i == 1
            P = [];
        elseif i > 1
            P = [per(i-1,1),per(i-1,2),per(i-1,3)];
        end
        per(i,:) = Monte_UE_pro(wifi_sta, C, signr{i}, dr{i}, i, step, P);
        %         per(i,:) = Monte_UE(wifi_sta, com, signr{i}, dr{i});
    end
elseif p == 1
    per = Monte_UE(wifi_sta, C, sign, d);
    %     P = [];
    %     per = Monte_UE_pro(wifi_sta, com, sign, d, p, step, P);
end


%% KF
D = 2;         
N = length(per(:,1));
per = per';
A = eye(D);                     
X_kf = zeros(D, N);                
z_kf = per(1:2,:);                 

%         x(:, 1) = z(:,1);            
P = eye(D);                    
Q = Y*eye(D);                
R = [O 0 ;
    0 0.5];                  

for k = 2:1:N
    X_kf(:,k) = A * X_kf(:,k-1);
    P = A * P * A' + Q;      
    H = eye(D);
    K = P*H' * inv(H*P*H' + R); 
    % 二维 x,y
    z_kf(:,k) = [per(1,k) + rand; per(2,k) + rand];
    X_kf(:,k) = x(:,k) + K * (z(:,k)-H*x(:,k));   
    P = (eye(D)-K*H) * P;                       
end

per = X_kf;


%% PDR+WiFi+PF
% [x,y] is the PDR output as state prediction
% per is the Wi-Fi output as observation
% Calculate the output step：
L = zeros(length(x)-1,1);
sita = zeros(length(x)-1,1); 
for i = 1:1:length(x)-1
    L(i) = norm([x(i+1)-x(i); y(i+1)-y(i)]);
    sita(i) = 180/pi*atan((y(i+1)-y(i))/(x(i+1)-x(i)));
end



%% initialization
pt = 1000; % particle number
% N_eff = pt/3; % effective particle

xp = rand(1,pt)*50 - 10;
yp = rand(1,pt)*20 - 5;

W0 = ones(1,pt) * (1/pt); % Initialization weights

T = 8;

Xpf = zeros(pt,T); % Particle Filter Estimation State
Ypf = zeros(pt,T);
W = zeros(pt,T);
% Xparticles = zeros(pt,T); 
% Xz = zeros(pt,T); 
% Yz = zeors(pt,T);

% Initial sampling for a given state and observation prediction：
Xpf(:,1) = xp;
Ypf(:,1) = yp;
W(:,1) = W0;
di = zeros(pt,1);

X_end = zeros(T,1);
Y_end = zeros(T,1);

% per = per + 1.5*randn(length(x),2);

for k = 2:1:T
    % state transfer
    Xpf(:,k) = Xpf(:,k-1) + L(k-1)*cos(sita(k-1));
    Ypf(:,k) = Ypf(:,k-1) + L(k-1)*sin(sita(k-1));
    
    % Calculation of weights
    sigw = 3; 
    for i = 1:1:pt
        di(i) = norm([Xpf(i,k)-per(k,1), Ypf(i,k)-per(k,2)]);
        W(i,k) = 1/(sqrt(2*pi*sigw^2)) * exp(-di(i)^2/(2*sigw));
    end
    
    W(:,k) = W(:,k)./sum(W(:,k));
    
    N_eff = 1/sum(W(:,k).^2); 
    
    while(N_eff < pt/5)
        W_max = maxk(W(:,k),10);
        
        pos_max = zeros(10,1);
        for ps = 1:1:10
            pos_max(ps) = find(W(:,k)==W_max(ps));
        end
        
        x_max = Xpf(pos_max,k-1);
        y_max = Ypf(pos_max,k-1);
        
        % resample
        [Xpf(:,k-1), Ypf(:,k-1)] = re_generate(x_max,y_max,5,pt);
        
        % outIndex = systematicR(W(:,k)');
        % Xpf(:,k) = Xpf(outIndex,k);
        % Ypf(:,k) = Ypf(outIndex,k);
        % W(:,k) = W(outIndex,k);
        
        sig = randn(pt,1);
        Xpf(:,k) = Xpf(:,k-1) + L(k-1)*cos(sita(k-1)) + sig;
        sig = randn(pt,1);
        Ypf(:,k) = Ypf(:,k-1) + L(k-1)*sin(sita(k-1)) + sig;
  
        for i = 1:1:pt
            di(i) = norm([Xpf(i,k)-per(k,1), Ypf(i,k)-per(k,2)]);
            W(i,k) = 1/(sqrt(2*pi*sigw^2)) * exp(-di(i)^2/(2*sigw));
        end
      
        W(:,k) = W(:,k)./sum(W(:,k));  
        N_eff = 1/sum(W(:,k).^2); 
        fprintf('k=%d, N_eff = %f \n',k,N_eff);
    end
    
    X_end(k) = sum(W(:,k).*Xpf(:,k));
    Y_end(k) = sum(W(:,k).*Ypf(:,k));
end

per = [X_end,Y_end];