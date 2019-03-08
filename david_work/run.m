clc;
clear all;

global V;
global TAU_S_MAX;
global TAU_H_MAX;

V         = 1;
TAU_S_MAX = 0.1;
TAU_H_MAX = 0.1;

% Initial Conditions
z1    = -10;
z2    = -10;
theta = pi/4;
tau_s = 0;
m_s   = 0;
tau_h = 0;
m_h   = 0;

z0 = [z1; z2; theta; tau_s; m_s; tau_h; m_h];

% Global variables


% simulation horizon
TSPAN=[0 5];
JSPAN = [0 60];

rule = 1;
options = odeset("RelTol",1e-6,"MaxStep",.001);
% simulate
[t,j,z] = HyEQsolver(@F,@G,@C,@D,...
z0,TSPAN,JSPAN,rule,options,"ode23t");



