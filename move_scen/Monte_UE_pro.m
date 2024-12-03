function [per] = Monte_UE_pro(wifi_sta, C, sign, d, p, step, P)
% Input：wifi_sta coordinates、C Total signals、
%       sign is the UE detection signal、d is the distance of the detected signal
% Output：per 定位UE坐标范围
% 输出：per 定位UE坐标

% Initialize the radius of the first point spread
step(1) = 30;

%% Detection of the presence of common signals ( at least 4 of them)
[C_com, idx_C, idx_sign] = intersect(C,sign); 
sta_com = wifi_sta(idx_C,:);    
d_com = d(idx_sign);    

xt = sta_com(:,1);
yt = sta_com(:,2);
zt = sta_com(:,3);

%% MC
N = 25;
K = 20;

err_min = zeros(N,10);
result = zeros(K,3);

for k = 1:1:K 
    if p == 1
        pt = 1000;
        rp = zeros(pt,length(C_com));
        err = zeros(pt,1);
        
        xp = rand(1,pt)*200 - 100;   
        yp = rand(1,pt)*200 - 100;
        zp = rand(1,pt)*200 - 100;
    elseif p > 1
        pt = 500;
        rp = zeros(pt,length(C_com));
        err = zeros(pt,1);

        [xp,yp,zp] = sta_Point(P(1),P(2),P(3),step(p),pt);
        step(p) = 0.1*step(p);
    end
    for j = 1:1:N
        for i = 1:1:pt
            for o = 1:1:length(C_com)
                rp(i,o) = sqrt((xp(i)-xt(o)).^2 + (yp(i)-yt(o)).^2 + (zp(i)-zt(o)).^2)';
            end
            
            err(i) = sum(sqrt((rp(i,:) - d_com').^2));
        end
        err_min(j,:) = mink(err,10); 
        [xp,yp,zp] = sta_generate_pro(xp,yp,zp,err,j,err_min,pt,step(p));
    end
    
    err_min10 = mink(err,10);
    per_pos = [];
    for sp = 1:1:10
        per_pos = [per_pos; find(err == err_min10(sp))];
    end
    per_pos = unique(per_pos);
    result(k,:) =  [mean(xp(per_pos)),mean(yp(per_pos)),mean(zp(per_pos))];
end

per = mean(result);
end

