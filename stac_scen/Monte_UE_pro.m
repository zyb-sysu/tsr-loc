function [result,per] = Monte_UE_pro(wifi_sta, C, sign, d, pt, N, K)
% Input：wifi_sta coordinates、C Total signals、
%       sign is the UE detection signal、d is the distance of the detected signal
% Output：per 定位UE坐标

%% Detection of the presence of common signals (and at least 4 of them)
[C_com, idx_C, idx_sign] = intersect(C,sign); 
sta_com = wifi_sta(idx_C,:); 
d_com = d(idx_sign);

% Select the 10 points where d_com is smaller for localization
Nmin = 10;
if length(d_com) > Nmin
    d_com_min = mink(d_com,Nmin);
    d_com_min1 = unique(d_com_min);
    com_idx = [];
    for i = 1:length(d_com_min1)
        com_idx = [com_idx; find(d_com == d_com_min1(i))];
    end
    sta_com_min = sta_com(com_idx,:);
    C_com = C_com(com_idx);
    C_com = C_com(1:Nmin);

    d_com = d_com_min;

    xt = sta_com_min(:,1);
    yt = sta_com_min(:,2);
    zt = sta_com_min(:,3);

else
    xt = sta_com(:,1);
    yt = sta_com(:,2);
    zt = sta_com(:,3);
end

% xt = sta_com(:,1);
% yt = sta_com(:,2);
% zt = sta_com(:,3);

%% MC
% pt = 5000;
% N = 25;
% K = 20;

rp = zeros(pt,length(C_com));
err = zeros(pt,1);
err_min = zeros(N,10);
result = zeros(K,3);
per_pos = zeros(10,1);

for k = 1:1:K
    xp = rand(1,pt)*200 - 100;
    yp = rand(1,pt)*200 - 100;
    zp = rand(1,pt)*200 - 100;
    for j = 1:1:N
        for i = 1:1:pt
            % Distance from each simulated per to a known coordinate sta
            for o = 1:1:length(C_com)
                rp(i,o) = sqrt((xp(i)-xt(o)).^2 + (yp(i)-yt(o)).^2 + (zp(i)-zt(o)).^2)';
            end
            err(i) = sum(sqrt((rp(i,:) - d_com').^2));
            %             err(i,:) = abs(sum(rp(i,:) - d_com'))';
        end
        err_min(j,:) = mink(err,10);
        %         [xp,yp] = per_generate(xp,yp,err,j,err_min);
        [xp,yp,zp] = sta_generate(xp,yp,zp,err,j,err_min,pt);
    end
    
    err_min10 = mink(err,10);
    for sp = 1:1:10
        per_pos(sp) = find(err == err_min10(sp));
    end
    result(k,:) =  [mean(xp(per_pos)),mean(yp(per_pos)),mean(zp(per_pos))];
end

%% save best
per = mean(result);
end

