%%
% Use current state to find a 
% control sequence that minimizes the objective function.
% Apply the 1st control 
%%
function u_out = mpc_controller4(z)

% General CPS globals
global TAU_S_MAX;
global V;
global ttc_list;
global obj_list;
global dis_list;


% Definition of state
z1        = z(1); % X Position
z2        = z(2); % Y Position
theta     = z(3); % Angle
tau_s     = z(4); % Sampler timer
m_s       = z(5); % Sampler memory
tau_h     = z(6); % Holder timer
m_h       = z(7); % Holder memory
tar_x     = z(8);
tar_y     = z(9);
obs_x     = z(10);
obs_y     = z(11);
obs_theta = z(12);


% Globals needed for optimization
global X0;
global TARGET;
global deltaT;

X0     = [z1;z2;theta];
TARGET = [tar_x; tar_y; 0];

global w1;
global w2;
global w3;
global w4;

% Weight of each component in objective function
w1 = .5;          % length
w2 = [1, 10, 20]; %last state distance 
w3 = 1;           %smoothness
w4 = -0.5;     %safety

global OBS_STATE0;
OBS_STATE0 = [obs_x; obs_y; obs_theta];


% Keep track of TTC 
global mem_TTC;
mem_TTC = [];

% Prediction Horizon Settings
global T;
global N;
T = 10; 
N = 20;


% Optimize to find u 
A = [];
b = [];
Aeq = [];
beq = [];
lb = -0.5 * ones(1, N);
ub = 0.5 * ones(1, N);
nonlcon = @circlecon;
rand_x0= rand(1, N);
options = optimoptions('fmincon','Algorithm','sqp','MaxIterations',10000);
%options=optimset('LargeScale','off','TolFun',.001,'MaxIter',1000,'MaxFunEvals',1000);
u_list = fmincon(@cost_fun,rand_x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

u_out = u_list(1);

end

function [c,ceq] = circlecon(u)

global X0;
global TARGET;
global V;
global L;
v = V;
l = L;

global OBS_STATE0;
global OBS_V;

global T;
global TTC;
global ttc_list;
c = [];


N = length(u);
deltaT = T/N;
X = zeros(3, N + 1);
X(:,1) = X0;

for i = 1:N
    X(:,i+1) = [v*cos(X(3,i))*deltaT; v*sin(X(3,i))*deltaT; v*deltaT*tan(u(i))/l]+X(:,i);
end

Xobs = zeros(3,N+1);
Xobs(:,1) = OBS_STATE0;
TTC = zeros(1,N);
for i = 1:N
    Xobs(:,i+1) = [OBS_V*cos(Xobs(3,i))*deltaT; OBS_V*sin(Xobs(3,i))*deltaT; OBS_V*deltaT*tan(0)/l]+Xobs(:,i);
end

vector1 = Xobs - X;
for i =1:N
    v1 = [v*cos(X(3,i)), v*sin(X(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    v2 = [OBS_V*cos(Xobs(3,i)), OBS_V*sin(Xobs(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    TTC(i) = min((norm(vector1(1:2,i)))/(v1-v2),100);
    if (TTC(i)<0)
        TTC(i) = 100;
    end
    
    c = [c (15 - TTC(i))];
end
ttc_list = [ttc_list TTC(1)];
 
%c = [c (15 - min(TTC))];
c = [];
ceq = [];

end


function cost = cost_fun(u)
%FUN Summary of this function goes here
%   Detailed explanation goes here
global X0;
global TARGET;
global V; % Ego velocity
global L; % Ego car length
global T; % Length of prediction horizon in seconds 
v = V;
l = L;

global w1;
global w2;
global w3;
global w4;

global OBS_STATE0;
global OBS_V;



N = length(u);
deltaT = T/N;
X = zeros(3, N + 1);
X(:,1) = X0;

for i = 1:N
    X(:,i+1) = [v*cos(X(3,i))*deltaT; v*sin(X(3,i))*deltaT; v*deltaT*tan(u(i))/l]+X(:,i);
end

%% cost1: efficiency(length)
vector1 = diff([X TARGET]')';
vector2 = sqrt(vector1(1,:).^2+vector1(2,:).^2);
cost1 = sum(vector2);


%% cost2: last state distance (distance to target state)
cost2 = X(:,N+1)-TARGET;

%% cost3: smoothness (jerk)
dX = diff(X')'/deltaT;
ddX = diff(dX')'/deltaT;
dddX = diff(ddX')'/deltaT;
cost3 = max(sqrt(dddX(2,:).^2)+sqrt(dddX(1,:).^2));

%% cost4: safety
Xobs = zeros(3,N+1);
Xobs(:,1) = OBS_STATE0;
TTC = zeros(1,N);
for i = 1:N
    Xobs(:,i+1) = [OBS_V*cos(Xobs(3,i))*deltaT; OBS_V*sin(Xobs(3,i))*deltaT; OBS_V*deltaT*tan(0)/l]+Xobs(:,i);
end

vector1 = Xobs - X;
for i =1:N
    v1 = [v*cos(X(3,i)), v*sin(X(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    v2 = [OBS_V*cos(Xobs(3,i)), OBS_V*sin(Xobs(3,i))]*vector1(1:2,i)/norm(vector1(1:2,i));
    TTC(i) = min((norm(vector1(1:2,i)))/(v1-v2),100);
    if (TTC(i)<0)
        TTC(i) = 100;
    end
 end
cost4 = min(TTC);

cost = sum([cost1*w1 norm(cost2'.*w2) cost3*w3 cost4*w4]);

end


