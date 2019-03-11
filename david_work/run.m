clc;
clear all;

global V; % Velocity of vehicle
global L; % Length of vehicle
global TAU_S_MAX;
global TAU_H_MAX;

V         = 10;
L         = 2.65;
TAU_S_MAX = 0.1;
TAU_H_MAX = 0.1;

% Initial Conditions
z1    = 0;
z2    = 0;
theta = 0;
tau_s = 0;
m_s   = 0;
tau_h = 0;
m_h   = 0;
tar_x   = 5 
tar_y   = -5;


z0 = [z1; z2; theta; tau_s; m_s; tau_h; m_h; tar_x; tar_y];

% Global variables


% simulation horizon
TSPAN=[0 5];
JSPAN = [0 60];

rule = 1;
options = odeset("RelTol",1e-6,"MaxStep",.001);
% simulate
[t,j,z] = HyEQsolver(@F,@G,@C,@D,...
z0,TSPAN,JSPAN,rule,options,"ode23t");



