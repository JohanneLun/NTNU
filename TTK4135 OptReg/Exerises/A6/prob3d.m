% Problem 3 d)

% System
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
Rt = 2*1.2;
R = sparse(kron(I_N, Rt));
G = blkdiag(Q, R);

% Equality constraint
Aeq_c1 = sparse(eye(N*nx));
Aeq_c2 = sparse(kron(diag(ones(N-1,1),-1), -A));
Aeq_c3 = sparse(kron(I_N, -B));
Aeq = [Aeq_c1 + Aeq_c2, Aeq_c3];

% Inequality constraints
x_lb = -Inf(N*nx,1);
x_ub =  Inf(N*nx,1);
u_lb = -ones(N*nu,1);
u_ub =  ones(N*nu,1);
lb = [x_lb; u_lb];
ub = [x_ub; u_ub];

u = NaN(nu,N);
x = NaN(nx,N+1);
x(:,1) = x0;

beq = [zeros(nx,1); zeros((N-1)*nx,1)];
% Solving the optimization problem at each time step
for t = 1:N
    beq(1:nx) = A*x(:,t);
    [z,fval,exitflag,output,lambda] = quadprog(G,[],[],[],Aeq,beq,lb,ub,[]);
    % Extracting variables
    u_ol = z(N*nx+1:N*nx+N*nu);	% Optimal open-loop control
    u(t) = u_ol(1);
    x(:,t+1) = A*x(:,t) + B*u(t); % simulation of next step
    
end
y = C*x;

% Plot
t_vec = 1:N; 

figure('Name','3 d)');
subplot(2,1,1);plot([0,t_vec],y);
grid('on');box('on');ylim([-0.5, 3]);ylabel('y_t');
subplot(2,1,2);plot(t_vec-1,u);
box('on');grid('on');ylim([-2, 1]);xlabel('t');ylabel('u_t');