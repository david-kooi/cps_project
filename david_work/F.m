function zdot = F(z)
%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
%--------------------------------------------------------------------------
% Project: Simulation of a hybrid system
% Description: Flow map
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00
global V;
global L;
global OBS_V 

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
obs_x   = z(10);
obs_y   = z(11);
obs_theta = z(12);

z1_dot        = V*cos(theta);
z2_dot        = V*sin(theta);
theta_dot     = V*tan(m_h)/L;
tau_s_dot     = 1;
m_s_dot       = 0;
tau_h_dot     = 1;
m_h_dot       = 0;
tar_x_dot     = V;
tar_y_dot     = 0;
obs_x_dot     = OBS_V*cos(obs_theta);
obs_y_dot     = OBS_V*sin(obs_theta);
obs_theta_dot = 0;

% Definition of zdot, with constant input
zdot = [z1_dot; z2_dot; theta_dot;...
        tau_s_dot; m_s_dot;...
        tau_h_dot; m_h_dot;...
        tar_x_dot; tar_y_dot;...
        obs_x_dot; obs_y_dot; obs_theta_dot];


end
