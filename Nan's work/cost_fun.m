function cost = cost_fun(u)
%FUN Summary of this function goes here
%   Detailed explanation goes here
global X0;
global T;
global Xtarg;
global w;
global v;
global l;

w1 = .5;
w2 = [1, 5, 10];
w3 = 1;
w = [w1, w2, w3];


N = length(u);
deltaT = T/N;
X = zeros(3, N + 1);
X(:,1) = X0;
l = 2.65;

for i = 1:N
    X(:,i+1) = [v*cos(X(3,i))*deltaT; v*sin(X(3,i))*deltaT; v*deltaT*tan(u(i))/l]+X(:,i);
end

%% cost1: efficiency(length)
vector1 = diff(X')';
vector2 = sqrt(vector1(1,:).^2+vector1(2,:).^2);
cost1 = sum(vector2);


%% cost2: (distance to target state)
cost2 = X(:,N+1)-Xtarg;

%% cost3: (jerk)
dX = diff(X')'/deltaT;
ddX = diff(dX')'/deltaT;
dddX = diff(ddX')'/deltaT;
cost3 = max(sqrt(dddX(2,:).^2)+sqrt(dddX(1,:).^2));

cost = sum([cost1*w1 norm(cost2'.*w2) cost3*w3]);
end

