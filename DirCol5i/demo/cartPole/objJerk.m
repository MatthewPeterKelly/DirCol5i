function normJerk = objJerk(X,dX,ddX,dddX,p)
% normAccel = objJerk(x,dx,ddx,dddx,p)
%
% This function computes the norm of the derivative of the
% acceleration of the pendulum bob
%
% INPUTS:
% 	x = position
% 	dx = velocity
% 	ddx = acceleration
%   p = parameter struct
%       .g = gravity
%       .m1 = cart mass
%       .m2 = pole mass
%       .l = pendulum length
%
% OUTPUTS:
%   normAccel = norm of the snap of the bob trajectory
%

%%%% Compute the norm of the accleration of the pendulum bob
q = X(2,:);
dq = dX(2,:);
ddq = ddX(2,:);
dddx = dddX(1,:);
dddq = dddX(2,:);
[~,normJerk2] = autoGen_normBobJerk(q,dq,ddq,dddx,dddq,p.l);

%%%% Rough scale factor:
normJerk = normJerk2*(10^-3);

end