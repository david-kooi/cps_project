
% Copyright 2015 The MathWorks, Inc.

function [c,ceq] = circlecon(u)

global X0;
global Xtarg;
global X;
global v;

global Xobs;
global Xobs0;
global vobs;

global T;
global l;
global TTC;
c = [];


N = length(u);
deltaT = T/N;
X = zeros(3, N + 1);
X(:,1) = X0;

% c = -1*ones(2*N,1);
c = -1*ones(1*N,1);

for i = 1:N
    X(:,i+1) = [v/cos(X(3,i))*cos(X(3,i))*deltaT; v/cos(X(3,i))*sin(X(3,i))*deltaT; v/cos(X(3,i))*deltaT*tan(u(i))/l]+X(:,i);
end

Xobs = zeros(3,N+1);
Xobs(:,1) = Xobs0;
TTC = zeros(1,N);
for i = 1:N
    Xobs(:,i+1) = [vobs*cos(Xobs(3,i))*deltaT; vobs*sin(Xobs(3,i))*deltaT; vobs*deltaT*tan(0)/l]+Xobs(:,i);
end

vector1 = Xobs - X;
for i =1:N
    v1 = [v/cos(X(3,i))*cos(X(3,i)), v/cos(X(3,i))*sin(X(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    v2 = [vobs*cos(Xobs(3,i)), vobs*sin(Xobs(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    TTC(i) = min((norm(vector1(1:2,i)))/(v1-v2),100);
    if (TTC(i)<0)
        TTC(i) = 100;
    end
    c(i) = 10 - TTC(i);
end

% diff_u = zeros(N-1,1);
% for i = 1:N-1
%     diff_u(i) = u(i+1)-u(i);
%     c(i+N) = 0.05 - abs(diff_u(i));
% end

ceq = [];
end
 
%c = [c (15 - min(TTC))];

