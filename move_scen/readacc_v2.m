function [ax,ay,az,time] = readacc_v2(filename,tp)

delimiterIn = ' ';
A = importdata(filename,delimiterIn);
La = length(A);
time = char(32*ones(La,14));
ax = zeros(La,1);
ay = zeros(La,1);
az = zeros(La,1);
for i = 1:La
    a = A{i};


    if tp == 1
        % scene 1
        tp0 = find(a=='【');
        tp1 = find(a==']');
        xp = find(a=='X');
        yp = find(a=='Y');
        zp = find(a=='Z');

        time(i,:) = a(tp0+1:tp1-1);
        ax(i) = str2double(a(xp+2:yp-2));
        ay(i) = str2double(a(yp+2:zp-2));
        az(i) = str2double(a(zp+2:end));

    elseif tp == 2
        % scene 2
        tp0 = find(a=='[');
        tp1 = find(a==']');
        xp = max(find(a=='X'));
        yp = max(find(a=='Y'));
        zp = max(find(a=='Z'));

        time(i,:) = a(tp0+1:tp1-2);
        ax(i) = str2double(a(xp+2:yp-2));
        ay(i) = str2double(a(yp+2:zp-2));
        az(i) = str2double(a(zp+2:end));
    end
end


%% Actual sampling rate varies, downsampling processing
if tp == 2
    charArray = time;
    result = [];

    count = 1;
    idx = 1;

    for i = 2 : length(charArray)
        if strcmp(charArray(i,:), charArray(i - 1,:))
            count = count + 1;
        else
            if count >= 6
                firstIdx = idx;
                lastIdx = i - 1;

                midIdx = floor((firstIdx + lastIdx) / 2);
                frontMidIdx = floor((firstIdx + midIdx) / 2);
                backMidIdx = floor((lastIdx + midIdx) / 2);

                result = [result; firstIdx, frontMidIdx, midIdx, backMidIdx, lastIdx];
            end
            count = 1;
            idx = i;
        end
    end
    if count >= 6
        firstIdx = idx;
        lastIdx = length(charArray);
        midIdx = floor((firstIdx + lastIdx) / 2);
        frontMidIdx = floor((firstIdx + midIdx) / 2);
        backMidIdx = floor((lastIdx + midIdx) / 2);
        result = [result; firstIdx, frontMidIdx, midIdx, backMidIdx, lastIdx];
    end

    pos_eff = reshape(result.', 1, [])';
    time = time(pos_eff,:);
    ax = ax(pos_eff);
    ay = ay(pos_eff);
    az = az(pos_eff);

end

end