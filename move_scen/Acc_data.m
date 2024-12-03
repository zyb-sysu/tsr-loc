%% **************  Acc_data  ************************
function [time_step,step] = Acc_data(all, time_com, turn_time)
%   Input：
%   Time-synchronized linear acceleration data：all
%   synchronized time series: time_com
%   turning time: turn_time

%   Output： 
%   time_step - Insert sampling time for turn times
%   step - Insert sampling distance for turn times

%% KF
x0 = 0; 
P0 = var(all); 
Q = 0.5; 
R0 = 1; 
[aLL, ~] = SimpleKalman(all, x0, P0, Q, R0);

%% Eliminate baseline drift
aLL_filt = wden(aLL, 'modwtsqtwolog', 's', 'mln', 1, 'db4');

%% Finding Peaks and Valleys
[~, t_all1] = findpeaks(aLL_filt);
[~, t_all2] = findpeaks(-aLL_filt); 
t_all1 = 0.2*t_all1;
t_all2 = 0.2*t_all2;

%% periodic number
% dt_all = diff(sort([t_all1 t_all2])); 
% f_all = ones(1,length(dt_all))./dt_all;

%% Convert the step size into distance, i.e. (x,y) coordinates
time_step = 0:5:5*(length(time_com)-1); % 5s

time_step = unique(sort([time_step turn_time])); 
step = zeros(1,length(time_step)-1);

for i = 1:length(time_step)-1
    t_all_trs1 = t_all1(time_step(i)<t_all1);
    t_all_trs1 = t_all_trs1(t_all_trs1<time_step(i+1));
    t_all_trs2 = t_all2(time_step(i)<t_all2);
    t_all_trs2 = t_all_trs2(t_all_trs2<time_step(i+1));
    
    dt_all_trs = diff(sort([t_all_trs1 t_all_trs2])); 
    f_all_trs = ones(1,length(dt_all_trs))./dt_all_trs;
    S_all_trs = 0.4504*f_all_trs - 0.1656; % 步长
    
    step(i) = sum(S_all_trs);
end

step = [0 step];

% figure;
% stem(time_step,step);
end