%% **************  函数readwifi  ************************
function [time,sign,RSSI] = readwifi_v2(filename,tp)
% Read wifi data and categorize data
% Output time, identifier, RSSI

fileID = fopen(filename,'r'); 
k=1;
while ~feof(fileID)     
    tline=fgetl(fileID);  
    if ~isempty(tline)    
        [~,n]=size(tline);
        for i=1:n
            data(k,i)=tline(i); 
        end
        k = k+1;
    end
end
fclose(fileID);

di = strtrim(cellstr(data));
pt0 = ~cellfun('isempty', di);
data = char(di(pt0));
k = size(data,1);

%%  Data classification
for i=1:1:k-1
    for j=2:15
        time(i,j-1)=data(i,j); 
    end

    if tp == 1
        % scene 1
        for j=23:39
            sign(i,j-22)=data(i,j); 
        end
        for j=48:50
            RSSI(i,j-47)=data(i,j); 
        end

    elseif tp == 2
        % scene 2
        for j=22:38
            sign(i,j-21)=data(i,j);
        end
        for j=46:48
            RSSI(i,j-45)=data(i,j); 
        end
    end
    
end

p0 = find(RSSI(:,1)=='0');
RSSI(p0,:) = [];
time(p0,:) = [];
sign(p0,:) = [];

if tp == 1
    sign = string([sign(:,1:2),sign(:,4:5),sign(:,7:8)...
                    ,sign(:,10:11),sign(:,13:14),sign(:,16:17)]);
    RSSI = str2num(RSSI(:,2:3));
elseif tp == 2
    sign = string([sign(:,1:2),sign(:,4:5),sign(:,7:8)...
                    ,sign(:,10:11),sign(:,13:14),sign(:,16:17)]);
    RSSI = str2num(RSSI(:,2:3));
end
end