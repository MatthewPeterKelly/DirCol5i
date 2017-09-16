function dObj = objFunJerk(z,dz,ddz,dddz,p)
% dObj = objFunJerk(z,dz,ddz,dddz,p)
%
% This function compute the norm of the jerk-squared of the wrist
% joint,scaled to be on the order of 1.
%
% INPUTS:
%   z = [2,1] = [q1;q2] = configuration vector
%   dz = [2,1] = [dq1;dq2] = configuration rate vector
%   ddz = [2,1] = [ddq1;ddq2] = acceleration vector
%   dddz = [2,1] = [dddq1;dddq2] = acceleration vector
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS: 
%   dOBj = C * norm(jerk of wrist)^2
%


[~,dddp2] = acrobotPosJerk(z,dz,ddz,dddz,p);
ddds2 = dddp2(1,:).^2 + dddp2(2,:).^2;
dObj = 1e-5*ddds2;

end