%%
% Use current state to find a 
% control sequence that minimizes the objective function.
% Apply the 1st control 
%%
function u_out = mpc_controller3(z)

global TAU_S_MAX;
global V;
global ttc_list;
global obj_list;
global dis_list;


% Definition of state
z1      = z(1); % X Position
z2      = z(2); % Y Position
theta   = z(3); % Angle
tau_s   = z(4); % Sampler timer
m_s     = z(5); % Sampler memory
tau_h   = z(6); % Holder timer
m_h     = z(7); % Holder memory
tar_x   = z(8);
tar_y   = z(9);

TARGET = [tar_x; tar_y; 0];




% Controller settings
nx = 3; % Number of states
nu = 1; % Number of inputs

Q = eye(nx); % Cost of z1,z2
R = 2;       % Cost of control
N = 7;       % Prediciton horizon

% State and control variables
%x = [z1;z2;theta]; % Initial plant state


x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));

traj_x      = []; 
constraints = [];
objective   = 0;
TTC_act     = sdpvar(1,1);
d           = sdpvar(N,1);

constraints = [constraints x{1} == [z1;z2;theta]];

% Create objective function
for k = 1:N 
     

    %cost1 = norm(TARGET-x{k});       % Distance to target 
    
 
    % Create TTC STL Constraints
    TTC = 1;
    obs_x = 3;
    obs_y = 0;
    phi   = sdpvar(1,1);
    
    % Jerk Constraints
    %dddx = x{k+3} - 3*x{k+2} - 3*x{k+1} - x{k};
    %dddx = dddx / TAU_S_MAX^3;
    %objective = objective + norm(dddx);
    %constraints = [constraints, dddx <= 5];
    


    % Model constraints
    model  = plant_model(x{k}, u{k}); % Update x
    constraints = [constraints, x{k+1} == model];
    %constraints = [constraints -1 <= u{k} <= 1];
    
    
    %objective = objective + abs(u{k});


    % TTC Constraints
    plant_pos  = [x{k+1}(1); x{k+1}(2)];
    obs_pos    = [10;5];
    d          = [obs_pos(1) - plant_pos(1); obs_pos(2) - plant_pos(2)];
    d_norm     = ((obs_pos(1) - plant_pos(1))^2 + (obs_pos(2) - plant_pos(2))^2 )^0.5;
    plant_v    = [V*cos(theta); V*sin(theta)];
    obs_v      = [0; 0];
    plant_vp   = (plant_v(1)*d(1) + plant_v(2)*d(2)) / d_norm;
    obs_vp     = 0;%(obs_v' * d) / (d(1)^2 + d(2)^2)^0.5; 
%
    TTC_act = d_norm / plant_vp; 
    if(value(TTC_act) < 0)
       TTC_act=100; 
    end
    constraints = [constraints TTC_act >= 2];

end

%cost = (x{N}(1) - TARGET(1))^2 + (x{N}(2) - TARGET(2))^2;
%cost = cost^0.5;
%cost = ((x{N}(1) - TARGET(1))^2 + (x{N}(2) - TARGET(2))^2)^0.5; 
objective  = objective + norm(x{N} - TARGET);


ops = sdpsettings('solver','fmincon');
ops.fmincon.MaxIter = 1000;
U = optimize(constraints,objective, ops);
ttc_list = [ttc_list value(TTC_act)];
obj_list = [obj_list value(objective)];
dis_list = [dis_list value(d(1))];
u_out = value(u{1});




end



function next_state = plant_model(x, theta_dot)
    global V;
    global TAU_S_MAX;
    global L;

    x_pos = x(1);
    y_pos = x(2);
    theta_sdp = x(3);

    theta_sdp = theta_sdp + TAU_S_MAX*V*tan(theta_dot)/L; 
    z1_next   = x_pos + TAU_S_MAX*V*cos(theta_sdp);
    z2_next   = y_pos + TAU_S_MAX*V*sin(theta_sdp); 
    

    next_state = [z1_next; z2_next; theta_sdp];
end


