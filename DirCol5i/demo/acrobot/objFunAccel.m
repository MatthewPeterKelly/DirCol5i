function dObj = objFunAccel(z,dz,ddz,p)
% dObj = objFunAccel(z,dz,ddz,p)
%
% This function compute the norm of the acceleration-squared of the wrist
% joint,scaled to be on the order of 1.
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
% OUTPUTS: 
%   dOBj = C * norm(acceleration of wrist)^2
%


[~,ddp2] = acrobotPosAccel(z,dz,ddz,p);
dds2 = ddp2(1,:).^2 + ddp2(2,:).^2;
dObj = 0.002*dds2;

end