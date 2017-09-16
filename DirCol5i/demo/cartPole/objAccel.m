function normAccel = objAccel(X,dX,ddX,p)
% normAccel = objAccel(x,dx,ddx,p)
%
% This function computes the norm of the acceleration of the pendulum bob
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
ddx = ddX(1,:);
ddq = ddX(2,:);
[normAccel1, normAccel2] = autoGen_normBobAccel(q,dq,ddx,ddq,p.l);
normAccel = normAccel1 + normAccel2;

%%%% Roughly scale the problem
normAccel = normAccel*0.002;  

end