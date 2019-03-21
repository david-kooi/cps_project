function cost = cost_fun(u)
%FUN Summary of this function goes here
%   Detailed explanation goes here
global X0;
global Xtarg;
global X;
global v;

global w1;
global w2;
global w3;
global w4;
global w5;
global w6;

global Xobs;
global Xobs0;
global vobs;

global T;
global l;
global TTC;


N = length(u);
deltaT = T/N;
X = zeros(3, N + 1);
X(:,1) = X0;

for i = 1:N
    X(:,i+1) = [v/cos(X(3,i))*cos(X(3,i))*deltaT; v/cos(X(3,i))*sin(X(3,i))*deltaT; v/cos(X(3,i))*deltaT*tan(u(i))/l]+X(:,i);
end

%% cost1: efficiency(length)
vector1 = diff([X Xtarg]')';
vector2 = sqrt(vector1(1,:).^2+vector1(2,:).^2);
cost1 = sum(vector2);


%% cost2: last state distance (distance to target state)
cost2 = X(:,N+1)-Xtarg;

%% cost3: smoothness (jerk)
dX = diff(X')'/deltaT;
ddX = diff(dX')'/deltaT;
dddX = diff(ddX')'/deltaT;
cost3 = max(sqrt(dddX(2,:).^2)+sqrt(dddX(1,:).^2));

%% cost4: safety
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
 end
cost4 = min(TTC);

%% cost5: vibrate
% diff_u = zeros(N-1,1);
% U = zeros(N+1,1);
% U(1) = 0;
% U(2:N+1) = u;
% for i = 1:N
%     diff_u(i) = norm(U(i+1)-U(i));
% end
% cost5 = sum(diff_u);

cost5 = sum(abs(diff(u)));
%% cost 6: starting action and ending action

cost6 = sum([abs(u(1)),abs(u(N))]);  

cost = sum([cost1*w1 norm(cost2'.*w2) cost3*w3 cost4*w4 cost5*w5 cost6*w6]);
end

