clc
clear all
close all

N = 100;

global X0;
global T;
global Xtarg;
global w;
global v;
global l;
v = 1;
T = 10;



X0 = [0, 2.5, 0]';
Xtarg = [10, 7.5, 0]';

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
x0 = rand(1, N);
%options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
u = fmincon(@cost_fun,x0,A,b);

X = zeros(3, N + 1);
X(:,1) = X0;
deltaT = T/N;

for i = 1:N
    X(:,i+1) = [v*cos(X(3,i))*deltaT; v*sin(X(3,i))*deltaT; v*deltaT*tan(u(i))/l]+X(:,i);
end

figure(1)
plot([X0(1) X(1,:) Xtarg(1)],[X0(2) X(2,:) Xtarg(2)],'-');
hold on
plot(X0(1),X0(2),'r*');
plot(Xtarg(1),Xtarg(2),'b*');
xlabel('x (m)');
ylabel('y (m)');
grid on
