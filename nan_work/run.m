clc
clear all
close all

global X0;
global Xtarg;
global X;
global v;

global w1;
global w2;
global w3;
global w4;

global Xobs;
global Xobs0;
global vobs;

global T;
global l;
global TTC;
%% ego-vehicle state
l = 2.65;
X0 = [0, 2.5, 0]';
Xtarg = [10, 7.5, 0]';
v = 1;

%% guest-vehicle state
Xobs0 = [10; 2.5; 0];
vobs = 0.5;

%% weight of each component in objective function
w1 = .5;
w2 = [1, 5, 10];
w3 = 1;
w4 = -0.5;

%% Receding Time Horizon setting
T = 10;
N = 100;

%% main
A = [];
b = [];
Aeq = [];
beq = [];
lb = -0.4922 * ones(1, N);
ub = 0.4922 * ones(1, N);
x0 = rand(1, N);
u = fmincon(@cost_fun,x0,A,b);

% X = zeros(3, N + 1);
% X(:,1) = X0;
% deltaT = T/N;

% for i = 1:N
%     X(:,i+1) = [v*cos(X(3,i))*deltaT; v*sin(X(3,i))*deltaT; v*deltaT*tan(u(i))/l]+X(:,i);
% end

figure(1)
plot([X0(1) X(1,:) Xtarg(1)],[X0(2) X(2,:) Xtarg(2)],'r-');
hold on
plot(Xobs(1,:),Xobs(2,:),'b-');
hold on
plot(X0(1),X0(2),'r*');
plot(Xtarg(1),Xtarg(2),'b*');
xlabel('x (m)');
ylabel('y (m)');
grid on

figure(2)
plot(TTC);
grid on
