%% Assignment 7, Problem 2 b)
%  Observer pole placement

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

% LQR:
Q = diag([4 4]);
R = 1;
[K,P,e] = dlqr(A,B,Q/2,R/2,[]);
A_cl = A - B*K; % Closed-loop matrix
p_sys_dlqr = eig(A_cl); % Poles in the closed-loop system (also returned in e by dlqr)

% Observer:
p_obs_d = 0.5 + 0.03j*[1; -1]; % Desired poles
KF = place(A',C',p_obs_d).'; % Resulting observer gain

% Simulate
tf = 50; % Final time step
x = NaN(2,tf+1);
u = NaN(1,tf+1);
y = NaN(1,tf+1);
x_hat = NaN(2,tf+1);

x0     = [5, 1]'; % Initial state
x0_hat = [6, 0]'; % Initial state estimate

x(:,1) = x0;
x_hat(:,1) = x0_hat;

for t = 1:tf
    % System simulated one step ahead:
    u(:,t) = -K*x_hat(:,t);
    x(:,t+1) = A*x(:,t) + B*u(:,t);
    y(:,t) = C*x(:,t);
    % Calculate state estimate based on measurement y:
    x_hat(:,t+1) = A*x_hat(:,t) + B*u(:,t) + KF*(y(:,t) - C*x_hat(:,t));
end

%% Plot
% t_vec = 0:tf; % Time vector
% 
% % Plot optimal trajectory
% figure(1);
% %subplot(2,1,1);
% plot(t_vec, x, '--', 'linewidth', 2); hold on;
% plot(t_vec, x_hat, '-'); hold off;
% hleg = legend('$x_1(t)$', '$x_2(t)$', '$\hat{x}_1(t)$', '$\hat{x}_2(t)$');
% set(hleg, 'Interpreter', 'Latex');
% grid('on');
% box('on');
% ylim([-4, 8]);
% ylabel('States and estimate');
% 
% figure(2)
% hold off;
% %subplot(2,1,2);
% plot(t_vec,u);
% box('on');
% grid('on');
% ylim([-8, 2]);
% ylabel('u_t');
% xlabel('t');

%% 
zero = [0 0
    0 0];
xi_11=A-B*K;
xi_12=B*K;
xi_22=A-KF*C;

eig_kf = eig(A-KF*C)

xi_tt = [xi_11 xi_12
    zero xi_22]
eigen = eig(xi_tt)



