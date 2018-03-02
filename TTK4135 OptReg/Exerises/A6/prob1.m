% Problem 1
clc; clear all; close all;

% d)
A = [1 0.5; 0 1];
b = [0.125; 0.5];
Q = 2*eye(2)    *0.5;
R = 2           *0.5;
N = [];

[K,P,e] = dlqr(A,b,Q,R,N);

disp(K);    disp(P);    disp(e);    disp(abs(e));