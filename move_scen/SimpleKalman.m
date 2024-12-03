%% ************** SimpleKalman *********************
function [X, P] = SimpleKalman(Y, x0, P0, Q, R)
% Simple one-dimensional Kalman filter
% Input parameters:
% Y: Observed data
% x0: Initial state estimation
% P0: Initial state covariance estimate
% Q: State noise covariance
% R: Observation noise covariance
% Output parameters:
% X: state estimate
% P: State covariance estimate

n = length(Y);
X = zeros(n,1);
P = zeros(n,1);
X(1) = x0;
P(1) = P0;

for k = 1:n-1
    x_pre = X(k); 
    P_pre = P(k) + Q; 
    K = P_pre / (P_pre + R); 
    X(k+1) = x_pre + K * (Y(k) - x_pre);
    P(k+1) = (1 - K) * P_pre;
end
end
