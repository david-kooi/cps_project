function cost = cost_fun(u)
%FUN Summary of this function goes here
%   Detailed explanation goes here
global X0;
global T;
global Xtarg;
global w;
global v;

N = length(u);
deltaT = T/N;
X = zeros(2, N + 1);
X(:,1) = X0;

for i = 1:N
    X(:,i+1) = [v*cos(u(i))*deltaT; v*sin(u(i))*deltaT]+X(:,i);
end

%% cost1: efficiency
vector1 = diff(X')';
vector2 = sqrt(vector1(1,:).^2+vector1(2,:).^2);
cost1 = sum(vector2);

%%


%% cost3: 
cost2 = norm(X(:,N+1)-Xtarg);
cost = [cost1 cost2]*w';
% cost = 100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
end
