%%
% Use current state to find a 
% control sequence that minimizes the objective function.
% Apply the 1st control 
%%
function u_out = mpc_controller(z)
persistent controller;


% Definition of state
z1      = z(1); % X Position
z2      = z(2); % Y Position
theta   = z(3); % Angle
tau_s   = z(4); % Sampler timer
m_s     = z(5); % Sampler memory
tau_h   = z(6); % Holder timer
m_h     = z(7); % Holder memory


if(isempty(controller))
    controller = create_controller();
end

current_state = [z1;z2;theta];

% Get control sequence
U = controller(current_state);
% Output only 1st control 
u_out = U(1);


function c = create_controller()

    % Controller settings
    nx = 3; % Number of states
    nu = 1; % Number of inputs

    Q = eye(nx); % Cost of z1,z2
    R = 2;       % Cost of control
    N = 7;       % Prediciton horizon

    % Set of state and control variables
    u   = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x0  = sdpvar(nx,1) 
    x   = x0;

    constraints = 0;
    objective   = 0;
    % Create objective function
    for k = 1:N 
        x            = plant_model(x, u{k});
        objective    = objective + x'*Q*x  + R*u{k}^2; 
        constraints  = [constraints, -1 <= u{k} <= 1]
    end

    ops = sdpsettings('verbose',2);
    c = optimizer(constraints,objective,ops,x0,u{1});
    


end

function next_state = plant_model(x, theta_dot)
    global V;
    global TAU_S_MAX;

    x_pos_sdp = x(1);
    y_pos_sdp = x(2);
    theta_sdp = x(3);

    z1_next = x_pos_sdp + TAU_S_MAX*V*cos(theta_sdp);
    z2_next = y_pos_sdp + TAU_S_MAX*V*sin(theta_sdp); 
    theta_sdp = theta_sdp + TAU_S_MAX*theta_dot; 

    next_state = [z1_next; z2_next; theta_sdp];
end

end
