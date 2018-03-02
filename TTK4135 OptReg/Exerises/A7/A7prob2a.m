%% Assignment 7, Problem 2 a)
%  Finds the LQR feedback gain matrix K using dlqr

k1 = 1;
k2 = 1;
k3 = 1;
T = 0.1;

% Continuous time:
Ac = [  0    1 
      -k1  -k2 ];
Bc = [0  k3]';
Cc = [0  1];

% Discrete time:
A = eye(2) + Ac*T;
B = Bc*T;
C = [1  0];

% sys_d = c2d(sys_c, T)
% 
% eig_d = eig(A)
% abs_eig_d = abs(eig_d)
% [zeros,poles,~] = zpkdata(sys_d);

% LQR:
Q = diag([4 4]);
R = 1;
[K,P,e] = dlqr(A,B,Q/2,R/2,[]);
A_cl = A - B*K; % Closed-loop matrix
p_sys_dlqr = eig(A_cl); % Poles in the closed-loop system (also returned in e by dlqr)
K