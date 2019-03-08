function v  = D(z) 
%--------------------------------------------------------------------------
% Matlab M-file Project: HyEQ Toolbox @  Hybrid Systems Laboratory (HSL), 
% https://hybrid.soe.ucsc.edu/software
% http://hybridsimulator.wordpress.com/
%--------------------------------------------------------------------------
% Project: Simulation of a hybrid system
% Description: Jump set
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   See also HYEQSOLVER, PLOTARC, PLOTARC3, PLOTFLOWS, PLOTHARC,
%   PLOTHARCCOLOR, PLOTHARCCOLOR3D, PLOTHYBRIDARC, PLOTJUMPS.
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.3 Date: 05/20/2015 3:42:00
%
% Check on jump conditions
% % E.g.,
% if (x(1) <= u(1)) && (x(2) <= 0)  % jump condition
%     v = 1;  % report jump
% else
%     v = 0;   % do not report jump
% end


global TAU_S_MAX;
global TAU_H_MAX;

% Definition of state
z1     = z(1); % X Position
z2     = z(2); % Y Position
theta  = z(3); % Angle
tau_s  = z(4); % Sampler timer
m_s    = z(5); % Sampler memory
tau_h  = z(6); % Holder timer
m_h    = z(7); % Holder memory

% Check jump condition
if ( (tau_s >= TAU_S_MAX) | (tau_h >= TAU_H_MAX) )
    v =1;
else
    v =0;
end

end

