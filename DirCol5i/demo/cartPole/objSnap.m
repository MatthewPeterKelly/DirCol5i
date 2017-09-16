function normSnap = objSnap(X,dX,ddX,dddX,ddddX,p)
% normAccel = objSnap(x,dx,ddx,dddx,ddddx,p)
%
% This function computes the norm of the second derivative of the
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
dddq = dddX(2,:);
ddddx = ddddX(1,:);
ddddq = ddddX(2,:);
[~,normSnap2] = autoGen_normBobSnap(q,dq,ddq,dddq,ddddx,ddddq,p.l);

%%%% Rough scale factor:
normSnap = normSnap2*(10^-6);

end