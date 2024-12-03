%% ************** Gyro_data *******************
function [turn_num, turn_time, turn_angle] = Gyro_data(tx, ty, tz)
%   Input time-synchronized 3-axis gyro data - tx, ty, tz
%   Output turn count, time and turn angle - turn_num; turn_time; turn_angle

%% Eliminate zero bias
% Read stationary data
% filename='陀螺仪静止数据.txt';
% [tbx,tby,tbz,~] = readacc(filename);

% mean
tbx = -1.389474334782609e-04;
tby = -2.518422108695652e-04;
tbz = 2.171053573913044e-04;

tx = tx - tbx;
ty = ty - tby;
tz = tz - tbz;

%% KF
x0 = 0;
Px0 = var(tx); 
Py0 = var(ty);
Pz0 = var(tz);
Q = 0.5;
R0 = 1; 

[tX, ~] = SimpleKalman(tx, x0, Px0, Q, R0);
[tY, ~] = SimpleKalman(ty, x0, Py0, Q, R0);
[tZ, ~] = SimpleKalman(tz, x0, Pz0, Q, R0);

%% Eliminate baseline drift
TX = wden(tX, 'modwtsqtwolog', 's', 'mln', 1, 'db4');
TY = wden(tY, 'modwtsqtwolog', 's', 'mln', 1, 'db4');
TZ = wden(tZ, 'modwtsqtwolog', 's', 'mln', 1, 'db4');

%% Using angular acceleration to determine if there is a turn
dt = 0.2;
atx = zeros(length(TX)-1,1);
aty = zeros(length(TY)-1,1);
atz = zeros(length(TZ)-1,1);
for i = 1:length(tx)-1
    atx(i) = (TX(i+1) - TX(i))/dt;
    aty(i) = (TY(i+1) - TY(i))/dt;
    atz(i) = (TZ(i+1) - TZ(i))/dt;
end


%% Determine if there is turning behavior (set threshold)
std_x = std(TX); 

[TX1, tph] = findpeaks(TX); 
[TX2, tpl] = findpeaks(-TX); 
tph = 0.2*tph;  
tpl = 0.2*tpl; 

txh = find(TX1 > 2.5*std_x);
txl = find(TX2 > 2.5*std_x);

TX1 = TX1(txh); tph = tph(txh);
TX2 = -TX2(txl); tpl = tpl(txl);

%% Determine the time of the points
if isempty(txh) == 0  
    p_left_val = zeros(1,length(txh));
    p_left_idx = zeros(1,length(txh));
    p_right_val = zeros(1,length(txh));
    p_right_idx = zeros(1,length(txh));
    
    for j = 1:1:length(txh)
        peak_idx = find(TX == TX1(j));
        for i = peak_idx:-1:1
            if TX(i) < 0 && TX(i+1) >= 0
                p_left_val(j) = TX(i+1);
                p_left_idx(j) = i+1;
                break;
            end
        end
        if p_left_idx(j) == 0
            p_left_idx(j) = 1;
        end
        for i = peak_idx:1:length(TX)
            if TX(i) >= 0 && TX(i+1) < 0
                p_right_val(j) = TX(i);
                p_right_idx(j) = i;
                break;
            end
        end
    end
    
    p_th_left = p_left_idx * 0.2;
    p_th_right = p_right_idx * 0.2;
    
    %% turn left
    t_rl = cell(1,length(txh));
    TX_rl = cell(1,length(txh));
    x_thl = zeros(1,length(txh));
    roll_left = zeros(1,length(txh));
    for i = 1:1:length(txh)
        t_rl{i} = p_th_left(i):0.2:p_th_right(i);
        TX_rl{i} = TX(p_left_idx(i):p_right_idx(i));
        x_thl(i) = trapz(t_rl{i}, TX_rl{i})';
        roll_left(i) = rad2deg(x_thl(i));
    end
    
else
    roll_left = []; 
end

if isempty(txl) == 0 
    v_left_val = zeros(1,length(txl));
    v_left_idx = zeros(1,length(txl));
    v_right_val = zeros(1,length(txl));
    v_right_idx = zeros(1,length(txl));
    
    for j = 1:1:length(txl)
        peak_idx = find(TX == TX2(j));
        for i = peak_idx:-1:1
            if TX(i) > 0 && TX(i+1) <= 0
                v_left_val(j) = TX(i+1);
                v_left_idx(j) = i+1;
                break;
            end
        end
         if v_left_idx(j) == 0
            v_left_idx(j) = 1;
        end
        
        for i = peak_idx:1:length(TX)
            if TX(i) <= 0 && TX(i+1) > 0
                v_right_val(j) = TX(i);
                v_right_idx(j) = i;
                break;
            end
        end
    end
    
    v_th_left = v_left_idx * 0.2;
    v_th_right = v_right_idx * 0.2;
    
    %% turn ringht
    t_rr = cell(1,length(txl));
    TX_rr = cell(1,length(txl));
    x_thr = zeros(1,length(txl));
    roll_right = zeros(1,length(txl));
    for i = 1:1:length(txl)
        t_rr{i} = v_th_left(i):0.2:v_th_right(i);
        TX_rr{i} = TX(v_left_idx(i):v_right_idx(i));
        x_thr(i) = trapz(t_rr{i}, TX_rr{i})';
        roll_right(i) = rad2deg(x_thr(i));
    end
    
else
    roll_right = [];
end


%% Angle
turn_num = length(txh) + length(txl); 
turn_time = [tph, tpl];   
turn_angle = [roll_left, roll_right];   


[turn_time, t_idx] = sort(turn_time);
turn_angle = turn_angle(t_idx);

% figure;
% subplot(211)
% stem(turn_time,turn_angle);

i = 1;
while i < turn_num    
    if turn_time(i+1) - turn_time(i) < 2
       turn_time(i) = [];
       turn_angle(i+1) = turn_angle(i) + turn_angle(i+1);
       turn_angle(i) = [];
       turn_num = turn_num - 1;
    end
    i = i+1;
end

%% Th:15~35-30；35~50-45；50~-90
for i = 1:turn_num
    if turn_angle(i) >=15 && turn_angle(i) <=35
        turn_angle(i) = 30;
    elseif turn_angle(i) >35 && turn_angle(i) <=50
        turn_angle(i) = 45;
    elseif turn_angle(i) >50
        turn_angle(i) = 90;
    elseif turn_angle(i) >= -35 && turn_angle(i) <= -15
        turn_angle(i) = -30;
    elseif turn_angle(i) >= -47 && turn_angle(i) < -35
        turn_angle(i) = -45;
    elseif turn_angle(i) < -47
        turn_angle(i) = -90;
    else
        turn_angle(i) = 0;
    end
end

% subplot(212)
% stem(turn_time,turn_angle);
end