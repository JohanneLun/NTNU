%% Assignment 7, Problem 3 a)
%  MPC and state estimation

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

N = 10; % Length of time horizon
nx = size(A,2); % number of states (equals the number of rows in A)
nu = size(B,2); % number of controls (equals the number of rows in B)

% Cost function
I_N = eye(N);
Qt = diag([4 4]);
Q = kron(I_N, Qt);
Rt = 1;
R = kron(I_N, Rt);
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = eye(N*nx);                         % Component 1 of A_eq
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);    % Component 2 of A_eq
Aeq_c3 = kron(I_N, -B);                     % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

% Inequality constraint
x_lb = -Inf(N*nx,1);    % Lower bound on x
x_ub =  Inf(N*nx,1);    % Upper bound on x
u_lb = -4*ones(N*nu,1); % Lower bound on u
u_ub =  4*ones(N*nu,1); % Upper bound on u
lb = [x_lb; u_lb];      % Lower bound on z
ub = [x_ub; u_ub];      % Upper bound on z

% Observer:
p_obs_d = 0.5 + 0.03j*[1; -1]; % Desired poles
KF = place(A',C',p_obs_d).'; % Resulting observer gain
%% MPC

opt = optimset('Display','off', 'Diagnostics','off', 'LargeScale','off', 'Algorithm', 'active-set');

tf = 50; % Final time step
x = NaN(2,tf+1);
u = NaN(1,tf+1);
y = NaN(1,tf+1);
x_hat = NaN(2,tf+1);

x0     = [5, 1]'; % Initial state
x0_hat = [6, 0]'; % Initial state estimate

x(:,1) = x0;
x_hat(:,1) = x0_hat;

% Initialize beq
beq = [zeros(nx,1); zeros((N-1)*nx,1)];

for t = 1:tf
    
    % Update equality constraint with latest state estimate 
    beq(1:nx) = A*x_hat(:,t);
    
    % Solve optimization problem
    [z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[],opt);
    
    % Extract optimal control from solution z
    u_ol = z(N*nx+1:N*nx+N*nu);    % Control, open-loop optimal
    u(t) = u_ol(1); % Only first element is used
    
    % Simulate system one step ahead
    x(:,t+1) = A*x(:,t) + B*u(t);
    
    % Measure
    y(:,t) = C*x(:,t);
    
    % Calculate state estimate based on measurement y
    x_hat(:,t+1) = A*x_hat(:,t) + B*u(:,t) + KF*(y(:,t) - C*x_hat(:,t));
end

%% Plot
t_vec = 0:tf; % Time vector

% Plot optimal trajectory
figure(1);
%subplot(2,1,1);
plot(t_vec, x, '--', 'linewidth', 2); hold on;
plot(t_vec, x_hat, '-'); hold off;
hleg = legend('$x_1(t)$', '$x_2(t)$', '$\hat{x}_1(t)$', '$\hat{x}_2(t)$');
set(hleg, 'Interpreter', 'Latex');
grid('on');
box('on');
ylim([-4, 8]);
ylabel('States and estimate');

figure(2);
%subplot(2,1,2);
plot(t_vec,u);
box('on');
grid('on');
ylim([-4, 4]);
ylabel('u_t');
xlabel('t');