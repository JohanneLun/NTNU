% Problem 3 b)
clc; clear all; close all;

A = [0 0 0; 0 0 1; 0.1 -0.79 1.78];
B = [1 0 0.1]';
C = [0 0 1];

x0 = [0 0 1]'; 
N = 30; 
blocks = 6; b_lenght = N/blocks;
nx = size(A,2); nu = size(B,2);

% Cost function
Qt = 2*diag([0, 0, 1]);
Q = kron(eye(N), Qt);
Rt = 2*1;
R = kron(b_lenght*eye(blocks), Rt);
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = eye(N*nx);
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);
ones_block = kron(eye(blocks),ones(b_lenght,1));
Aeq_c3 = kron(ones_block, -B);
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

beq = [A*x0; zeros((N-1)*nx,1)];

% Inequality constraints
x_lb = -Inf(N*nx,1);
x_ub =  Inf(N*nx,1);
u_lb = -ones(blocks*nu,1);
u_ub =  ones(blocks*nu,1);
lb = [x_lb; u_lb];
ub = [x_ub; u_ub];

% Solving QP
[z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[]);

% Extracting variables
y = [x0(3); z(nx:nx:N*nx)];
u_blocks = z(N*nx+1:N*nx+blocks*nu);
u = ones_block*u_blocks; 

t = 1:N;

% Plot optimal trajectory
figure('Name','3 b)');
subplot(2,1,1); plot([0,t],y);grid('on');ylabel('y_t')
subplot(2,1,2);plot(t-1,u);grid('on');xlabel('t');ylabel('u_t');