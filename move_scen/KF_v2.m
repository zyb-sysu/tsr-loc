function [data_kf] = KF_v2(data)
x0 = data(1); 
P0 = var(data);
Q = 0.5; 
R0 = 0.5; 

[aLL, ~] = SimpleKalman(data, x0, P0, Q, R0); 
aLL = wden(aLL, 'modwtsqtwolog', 's', 'mln', 1, 'db4');
data_kf = aLL';
end