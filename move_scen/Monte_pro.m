%% ************** Monte *********************
function [result,sta_best,STA,C_STA] = Monte_pro(C,cla,pt,N,K,x,y,z,r)
%% Main Input Parameters：
% C for shared sign
% cla for Wifi data was collected times
% x,y,z for Initialized Motion Track Inputs
% r is the distance from the actual detected sign in C to each (x,y,z)

%% Main Output Parameters：：
% result for each simulation
% sta_best is the value that corresponds to the actual measurement result
% r_err_min Minimum distance error for each signal output

%% pre-allocated memory
rp = cell(1,length(C));
err = cell(1,length(C));
err_min = cell(1,length(C));
err_min10 = cell(1,length(C));
result = cell(1,length(C));
sta_pos = cell(1,length(C));

xp = cell(1,length(C));
yp = cell(1,length(C));
zp = cell(1,length(C));

dp = zeros(cla,length(C));
r_err = zeros(K,length(C));

for i = 1:length(C)
    rp{i} = zeros(pt,cla);
    err{i} = zeros(pt,1);
    err_min{i} = zeros(N,10);
    result{i} = zeros(K,3);
    sta_pos{i} = zeros(10,1);
end

%% MC
for k = 1:1:K 
    for i = 1:length(C)
        xp{i} = rand(1,pt)*100 - 50; 
        yp{i} = rand(1,pt)*100 - 50;
        zp{i} = rand(1,pt)*100 - 50;
    end
    for j = 1:1:N
        for i = 1:1:pt
            for o = 1:length(C)
                rp{o}(i,:) = sqrt((xp{o}(i)-x).^2 + (yp{o}(i)-y).^2 + (zp{o}(i)-z).^2);

                for c = 1:cla
                    err{o}(i) = err{o}(i) + sqrt((rp{o}(i,c) - r{c}(o)).^2);
                end
            end
        end
        
        for o = 1:length(C)
            err_min{o}(j,:) = mink(err{o},10);
            [xp{o},yp{o},zp{o}] = sta_generate(xp{o},yp{o},zp{o},err{o},j,err_min{o},pt);
            if j < N
                err{o} = zeros(pt,1);
            end
        end
    end
    
    for o = 1:length(C)
        err_min10{o} = mink(err{o},10);
        for sp = 1:1:10
            sta_pos{o}(sp) = find(err{o} == err_min10{o}(sp));
        end
    end
    
    r_ref = cell2mat(r(1:cla))';
    for o = 1:length(C)
        wt = 1./(err_min10{o});
        result{o}(k,:) = [sum(wt'.*xp{o}(sta_pos{o}))/sum(wt), sum(wt'.*yp{o}(sta_pos{o}))/sum(wt), sum(wt'.*zp{o}(sta_pos{o}))/sum(wt)];
        dp(:,o) = sqrt((result{o}(k,1)-x).^2 + (result{o}(k,2)-y).^2 + (result{o}(k,3)-z).^2);
    end
    r_err(k,:) = sum(sqrt((dp - r_ref).^2))';
end

%% optimal
% Method 1: Taking a weighted average of a number of times
sta_best = cell(1,length(C));
w_err = 1./r_err;

for i = 1:length(C)
    sta_best{i} = sum(w_err(:,i).*result{i})/sum(w_err(:,i));
end


% Method 2: Take the optimal result of many times and filter all the signals
r_err_min = min(r_err);
re_c_pos = zeros(1,length(C));
for i = 1:length(C)
    re_c_pos(i) = find(r_err(:,i) == r_err_min(i));
end
err_tb = find(r_err_min <= cla*2);
STA = zeros(length(err_tb),3);
for i = 1:length(err_tb)
    STA(i,:) = result{err_tb(i)}(re_c_pos(err_tb(i)),:);
end
C_STA = C(err_tb);


end