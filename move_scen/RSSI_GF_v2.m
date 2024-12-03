function [C, RSSI_filted] = RSSI_GF_v2(filename)

[time_wifi,sign,RSSI] = readwifi_v2(filename,2);

%% grouping
p = 1;
for i=1:length(time_wifi)-1
    if strcmp (time_wifi(i,:), time_wifi(i+1,:)) == 0 
        pos(p) = i; 
        p = p+1;    
    end
end

cla = p;
RSSIr = cell(1,cla);
signr = cell(1,cla);
for i = 0:1:cla-1
    if i == 0
        RSSIr{1} = RSSI(1:pos(1));
        signr{1} = sign(1:pos(1));
    elseif i == cla-1
        RSSIr{i+1} = RSSI(pos(i)+1:end);
        signr{i+1} = sign(pos(i)+1:end);
    else
        RSSIr{i+1} = RSSI(pos(i)+1:pos(i+1));
        signr{i+1} = sign(pos(i)+1:pos(i+1));
    end
end

%% common singal
C = signr{1};   
% CAP = cell(1,1);    

for i = 1:1:cla-1
    C = intersect(C,signr{i+1});
end



RSSI = zeros(length(C),cla); 
for i = 1:cla
    for j = 1:1:length(C)
        pos = find(signr{i} == C(j));
        RSSI(j,i) = -1 * RSSIr{i}(pos);
    end
end

% figure(1)
% for i = 1:cla
%     plot(RSSI(i,:));
%     hold on;
% end


%% GF
r = 5;

RSSI_filted = zeros(length(C),cla);
for i = 1:length(C)
    sigma    = std(RSSI(i,:));
    if sigma == 0
        RSSI_filted(i,:) = RSSI(i,:);
    else
        RSSI_filted(i,:) = GF(r, sigma, RSSI(i,:));
    end

%     figure(2)
%     plot(RSSI_filted(i,:));
%     hold on;
end
end

