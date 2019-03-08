yalmip('clear')
clear all

% Model data
A = [2 -1;1 0.2];
B = [1;0];
nx = 2; % Number of states
nu = 1; % Number of inputs

% MPC data
Q = eye(2);
R = 2;
N = 7;

% Initial state
x0 = [3;1];

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));

constraints = [];
objective = 0;
x = x0;
for k = 1:N
 x = A*x + B*u{k};
 objective = objective + norm(Q*x,1) + norm(R*u{k},1);
 constraints = [constraints, -1 <= u{k}<= 1];
end

optimize(constraints,objective);
value(u{1})


