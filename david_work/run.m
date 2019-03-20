clc;
clear all;

global V;         % Velocity of vehicle
global OBS_V;     % Velocity of guest-vehicle
global L;         % Length of vehicle
global TAU_S_MAX; % Sample time
global TAU_H_MAX; % Hold time
global ttc_list;  % List of ttc
global obj_list;  % List of objective function values
global dis_list;  % List of distances from guest-vehicle
global u_list;

V         = 1;
OBS_V     = 0.5;
L         = 2.65;
TAU_S_MAX = 0.1;
TAU_H_MAX = 0.1;
ttc_list = [];
obj_list = [];
dis_list = [];
u_list   = [];

% Initial Conditions
z1    = 0;
z2    = 2.5;
theta = 0;
tau_s = 0;
m_s   = 0;
tau_h = 0;
m_h   = 0;
tar_x   = 20; 
tar_y   = 5;
obs_x   = 10;
obs_y   = 2.5;
obs_theta = 0;


z0 = [z1; z2; theta; tau_s; m_s; tau_h; m_h; tar_x; tar_y; obs_x; obs_y; obs_theta];


% simulation horizon
TSPAN=[0 60];
JSPAN = [0 10000000000000];

rule = 1;
options = odeset("RelTol",1e-6,"MaxStep",.001);
% simulate
[t,j,z] = HyEQsolver(@F,@G,@C,@D,...
z0,TSPAN,JSPAN,rule,options,"ode23t");

 hold on
 %circle_plot(10,5,1);
 hold on
 plot(z(:,1),z(:,2)); % Ego-Vehicle
 hold on;
 plot(z(:,8),z(:,9)); % Reference
 plot(z(:,10),z(:,11)); % Guest-Vehicle
 
 
% 
% 
% 
% function h = circle_plot(x,y,r)
%     hold on
%     th = 0:pi/50:2*pi;
%     xunit = r * cos(th) + x;
%     yunit = r * sin(th) + y;
%     h = plot(xunit, yunit);
%     hold off
% end


