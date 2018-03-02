% Problem 3 e)

% System
A = [0 0 0; 0 0 1; 0.1 -0.79 1.78];
B = [1 0 0.1]';
C = [0 0 1];

x0 = [0 0 1]'; 
N = 30; 
nx = size(A,2); nu = size(B,2);

b_length = [1, 1, 2, 4, 8, 14]';
nb = numel(b_length);

% Cost function
I_N = eye(N);
Qt = 2*diag([0, 0, 1]);
Q = kron(I_N, Qt);
Rt = 2*1;
R = kron(diag(b_length), Rt);
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = eye(N*nx);
Aeq_c2 = kron(diag(ones(N-1,1),-1), -A);
ones_block = blkdiag(ones(b_length(1),1), ...
                     ones(b_length(2),1), ...
                     ones(b_length(3),1), ...
                     ones(b_length(4),1), ...
                     ones(b_length(5),1), ...
                     ones(b_length(6),1));
Aeq_c3 = kron(ones_block, -B);
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

% Inequality constraint
x_lb = -Inf(N*nx,1);
x_ub =  Inf(N*nx,1);
u_lb = -ones(nb*nu,1);
u_ub =  ones(nb*nu,1);
lb = [x_lb; u_lb];
ub = [x_ub; u_ub];

% MPC
u = NaN(nu,N);
x = NaN(nx,N+1);
x(:,1) = x0;

beq = [zeros(nx,1); zeros((N-1)*nx,1)];
for t = 1:N
    beq(1:nx) = A*x(:,t);
    % Solving QP
    [z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[]);
    
    % Extracting variables
    u_blocks = z(N*nx+1:N*nx+nb*nu);
    u_ol = ones_block*u_blocks;
    u(t) = u_ol(1);
    
    % Simulate system one step ahead
    x(:,t+1) = A*x(:,t) + B*u(t);
end

y = C*x;

t_vec = 1:N; 

figure('Name','3 d)');
subplot(2,1,1);plot([0,t_vec],y);
grid('on');box('on');ylim([-0.5, 3]);ylabel('y_t');
subplot(2,1,2);plot(t_vec-1,u);
box('on');grid('on');ylim([-2, 1]);xlabel('t');ylabel('u_t');