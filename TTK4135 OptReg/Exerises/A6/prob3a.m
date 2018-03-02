% Problem 3 a)
clc; clear all; close all;

A = [0 0 0; 0 0 1; 0.1 -0.79 1.78];
B = [1 0 0.1]';
C = [0 0 1];

x0 = [0 0 1]'; 
N = 30; 
nx = size(A,2); % dimension of x
nu = size(B,2); % dimension of u

% Cost function
I_N = sparse(eye(N));
Qt = 2*diag([0, 0, 1]);
Q = sparse(kron(I_N, Qt));
Rt = 2*1;
R = sparse(kron(I_N, Rt));
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = sparse(eye(N*nx));                         % Component 1 of A_eq
Aeq_c2 = sparse(kron(diag(ones(N-1,1),-1), -A));   % Component 2 of A_eq
Aeq_c3 = sparse(kron(I_N, -B));                    % Component 3 of A_eq
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

beq = sparse([A*x0; zeros((N-1)*nx,1)]);

% Inequality constraints
x_lb = -Inf(N*nx,1);
x_ub =  Inf(N*nx,1);
u_lb = -ones(N*nu,1);
u_ub =  ones(N*nu,1);
lb = [x_lb; u_lb];
ub = [x_ub; u_ub];

%% 3 a)
opt = optimset('Display','notify', 'Diagnostics','off', 'LargeScale','off');
[z_3,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[],opt);

% Extracting variables
y_3 = [x0(3); z_3(nx:nx:N*nx)];
u_3 = z_3(N*nx+1:N*nx+N*nu);

t = 1:N;

% Plot optimal trajectory
figure('Name','3 a)');
subplot(2,1,1); plot([0,t],y_3);grid('on');ylabel('y_t')
subplot(2,1,2);plot(t-1,u_3);grid('on');xlabel('t');ylabel('u_t');