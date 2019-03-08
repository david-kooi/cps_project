clc;
clear all;


global TAU_S_MAX;
global V;
V = 1;
TAU_S_MAX= 0.1;

z0 = [1;1;0];
z_list = [];
t_list  = [];

z = z0;
while(abs(z(1)) > 0.5 || abs(z(2)) > 0.5)
    z_list = [z_list z];
    t_list = [t_list i*TAU_S_MAX];
    
    u = mpc_controller_test(z);
    z = plant_model(z, u); 
end

a=1;

function next_state = plant_model(x, theta_dot)
    global V;
    global TAU_S_MAX;

    x_pos     = x(1);
    y_pos     = x(2);
    theta_sdp = x(3);

    theta_sdp = theta_sdp + TAU_S_MAX*theta_dot; 
    z1_next   = x_pos + TAU_S_MAX*V*cos(theta_sdp);
    z2_next   = y_pos + TAU_S_MAX*V*sin(theta_sdp); 


    next_state = [z1_next; z2_next; theta_sdp];
end