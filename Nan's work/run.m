clc
clear all
close all

N = 100;

global X0;
global T;
global Xtarg;
global w;
global v;

v = 1;
T = 10;
X0 = [0, 2.5]';
w = [1, 1];
Xtarg = [10, 7.5]';

A = [];
b = [];
x0 = rand(1, N);
u = fmincon(@cost_fun,x0,A,b);

X = zeros(2, N + 1);
X(:,1) = X0;
deltaT = T/N;

for i = 1:N
    X(:,i+1) = [v*cos(u(i))*deltaT; v*sin(u(i))*deltaT]+X(:,i);
end

figure(1)
plot(X(1,:),X(2,:),'-')
grid on
