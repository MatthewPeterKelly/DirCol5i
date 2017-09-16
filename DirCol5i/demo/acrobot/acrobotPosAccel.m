function [ddp1,ddp2] = acrobotPosAccel(z,dz,ddz,p)
% [ddp1,ddp2] = acrobotPosAccel(z,dz,ddz,p)
%
% This function computes the mechanical energy of the acrobot.
%
% INPUTS:
%   z = [2,1] = [q1;q2] = configuration vector
%   dz = [2,1] = [dq1;dq2] = configuration rate vector
%   ddz = [2,1] = [ddq1;ddq2] = acceleration vector
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS:ls
%   dp1 = [2,n] = acceleration of the elbow joint
%   dp2 = [2,n] = acceleration of the wrist
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

[ddp1,ddp2] = autoGen_acrobotPosAccel(q1,q2,dq1,dq2,ddq1,ddq2,p.l1,p.l2);

end