function zplus = G(z)
global u_list;
global TAU_S_MAX;
global TAU_H_MAX;

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



z1_plus    = z1;
z2_plus    = z2;
theta_plus = theta;
tau_s_plus = 1;
m_s_plus   = 0;
tau_h_plus = 1;
m_h_plus   = 0;
tar_x_plus = tar_x;
tar_y_plus = tar_y;
obs_x_plus = obs_x;
obs_y_plus = obs_y;
obs_theta_plus = obs_theta;

if(tau_s >= TAU_S_MAX)
    % Sample, calculate the control, and save control
    % until hold is ready to receive
    m_s_plus   = mpc_controller4(z);
    tau_s_plus = 0;
    
    u_list = [u_list m_s_plus];
    
    m_h_plus   = m_h;
    tau_h_plus = tau_h;
elseif(tau_h >= TAU_H_MAX)
    % Apply saved control
    m_h_plus   = m_s;
    tau_h_plus = 0;
    
    m_s_plus   = m_s;
    tau_s_plus = tau_s;  
end
    

zplus = [z1_plus; z2_plus; theta_plus;...
        tau_s_plus; m_s_plus;...
        tau_h_plus; m_h_plus;
        tar_x_plus; tar_y_plus;
        obs_x_plus; obs_y_plus; obs_theta_plus];


end
