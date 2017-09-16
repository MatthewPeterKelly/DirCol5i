function zero = pendulum(q,dq,ddq,u)
% zero = pendulum(q,dq,ddq,u)
%
% Computes the dynamics of a simple pendulum in second-order form
%
% INPUTS: 
%   q = angle
%   dq = rate
%   ddq = acceleration
%   u = torque
% 
% OUTPUTS:
%   dv = acceleration
%

k = 1.0;   % gravity 
b = 0.1;   % damping

% Dynamics
accel = -b*dq - k*sin(q) + u;
zero = accel - ddq;

end