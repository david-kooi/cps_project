%%
% Use current state to find a 
% control sequence that minimizes the objective function.
% Apply the 1st control 
%%
function u_out = mpc_controller2(z)
global TAU_S_MAX;
global V;


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
x = [z1;z2;theta]; % Initial plant state
u = sdpvar(repmat(nu,1,N),repmat(1,1,N));

traj_x      = []; 
constraints = [];
objective   = 0;


% Create objective function
for k = 1:N 
     
    x = plant_model(x, u{k}); % Update x
 
    % Create TTC STL Constraints
    TTC = 1;
    obs_x = 3;
    obs_y = 0;
    phi   = sdpvar(1,1);
    
    plant_pos = [x(1); x(2)];
    obs_pos  = [obs_x;obs_y];
    d         = plant_pos - obs_pos;
    plant_v   = [V*cos(x(3)); V*sin(x(3))];
    obs_v    = [0; 0];
    plant_vp  = ((plant_v' * d) * d) / norm(d);
    obs_vp   = (obs_v' * d) / norm(d);

    TTC_act = norm(d) / norm((plant_vp)); 
    
    
    constraints  = [constraints ];


    cost1 = norm(TARGET-x);       % Distance to target 
    objective  = objective + cost1; 
    traj_x = [traj_x x];
    


end

ops = sdpsettings('verbose',2);
u = optimize(constraints,objective);
u_out = value(u{1});


function next_state = plant_model(x, theta_dot)
    global L;

    x_pos = x(1);
    y_pos = x(2);
    theta_sdp = x(3);

    theta_sdp = theta + TAU_S_MAX*V*tan(theta_dot)/L; 
    z1_next   = x_pos + TAU_S_MAX*V*cos(theta_sdp);
    z2_next   = y_pos + TAU_S_MAX*V*sin(theta_sdp); 
    

    next_state = [z1_next; z2_next; theta_sdp];
end

end
