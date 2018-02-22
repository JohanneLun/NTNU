% Simulate RL circuit
t = [0 5]; h = 0.01;
i0 = 1; L = 1; R = 2; Kp = 1; iref = 2;

% Number of iterations
N = t(2)/h;

% Define Butcher-array (book p. 528)
A = diag([0.5 0.5 1]);
b = [1/6 2/6 2/6 1/6]';
c = [0 0.5 0.5 1]';

% Order of ERK method
sigma = size(A,1) + 1;

% RL-circuit model
RL = @(i) ( -R/L * i + 1/L * ( R*iref - Kp*(i - iref)) );

% Initialize storage
y = zeros(N+1,1); y(1) = i0;

% We use numerical scheme from book p. 526 
k = zeros(sigma,1);
for n = 1:N,
    for j = 1:sigma,
        A_diag = diag(A);
        k(j) = RL(y(n) + h * sum(A_diag(1:j - 1) .* k(1:j - 1)));
    end
    y(n + 1) = y(n) + h * sum(b .* k);
end

plot(t(1):h:t(2),y); xlabel('Time [s]'); ylabel('Current [A]');
