function zero = acrobotDynamics(z,dz,ddz,u,p)
% zero = acrobotDynamics(z,dz,ddz,u,p)
%
% This function computes the implicit second-order dynamics of the acrobot
% system, modeled as a double pendulum with two point masses, torque motor 
% between links, no friction.
%
% INPUTS:
%   z = [2,1] = [q1;q2] = configuration vector
%   dz = [2,1] = [dq1;dq2] = configuration vector
%   ddz = [2,1] = [ddq1;ddq2] = configuration vector
%   u = [1,1] = input torque (between links)
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS:
%   zero = [2,1] = equations of motion (constrain to zero)
% 
% NOTES:
%   
%   states:
%       1 = q1 = first link angle
%       2 = q2 = second link angle
%       3 = dq1 = first link angular rate
%       4 = dq2 = second link angular rate
%
%   angles: measured from negative j axis with positive convention
%

q1 = z(1,:);
q2 = z(2,:);
dq1 = dz(1,:);
dq2 = dz(2,:);
ddq1 = ddz(1,:);
ddq2 = ddz(2,:);

% Vectorized call to the dynamics
[eqn1,eqn2] = autoGen_implicitDynamics(q1,q2,dq1,dq2,ddq1,ddq2,u,...
    p.m1, p.m2, p.g, p.l1, p.l2);
zero = [eqn1; eqn2];

end