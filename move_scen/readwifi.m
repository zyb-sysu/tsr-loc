%% **************  readwifi  ************************
function [time,sign,RSSI] = readwifi(filename)
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

%% Data classification
for i=1:1:k-1
    for j=2:15
        time(i,j-1)=data(i,j); 
    end
    for j=23:39
        sign(i,j-22)=data(i,j); 
    end
    for j=48:50
        RSSI(i,j-47)=data(i,j); 
    end
end
RSSI = str2num(RSSI(:,2:3));

sign = string([sign(:,1:2),sign(:,4:5),sign(:,7:8)...
        ,sign(:,10:11),sign(:,13:14),sign(:,16:17)]);
end