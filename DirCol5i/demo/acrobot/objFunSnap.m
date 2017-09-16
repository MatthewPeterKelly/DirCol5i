function dObj = objFunSnap(z,dz,ddz,dddz,ddddz,p)
% dObj = objFunSnap(z,dz,ddz,dddz,ddddz,p)
%
% This function compute the norm of the jerk-squared of the wrist
% joint,scaled to be on the order of 1.
%
% INPUTS:
%   z = [2,1] = [q1;q2] = configuration vector
%   dz = [2,1] = [dq1;dq2] = configuration rate vector
%   ddz = [2,1] = [ddq1;ddq2] = acceleration vector
%   dddz = [2,1] = [dddq1;dddq2] = jerk vector
%   dddz = [2,1] = [ddddq1;ddddq2] = snap vector
%   p = parameter struct:
%       .m1 = elbow mass
%       .m2 = wrist mass
%       .g = gravitational acceleration
%       .l1 = length shoulder to elbow
%       .l2 = length elbow to wrist
%
% OUTPUTS: 
%   dOBj = C * norm(snap of wrist)^2
%

[~,ddddp2] = acrobotPosSnap(z,dz,ddz,dddz,ddddz,p);
dddds2 = ddddp2(1,:).^2 + ddddp2(2,:).^2;
dObj = 1e-6*dddds2;

end