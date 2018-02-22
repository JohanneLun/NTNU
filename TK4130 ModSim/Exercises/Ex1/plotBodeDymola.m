% load output from Dymola linearize
load dslin
% ABCD is A, B, C and D matrix stacked into one matrix
% nx is number of states (dimension of the A matrix)

A = ABCD(1:nx,1:nx); B = ABCD(1:nx,nx+1:end);
C = ABCD(nx+1:end,1:nx); D = ABCD(nx+1:end,nx+1:end);

% Plot Bode response
bode(A,B,C,D)