function eqns = cartPoleDynamics(x,dx,ddx,u,p)
% eqns = cartPoleDynamics(x,dx,ddx,u,p)
%
% This function computes the first-order dynamics of the cart-pole.
%
% INPUTS:
%   t = time
% 	x = position
% 	dx = velocity
% 	ddx = acceleration
%  	u = control
%   p = parameter struct
%       .g = gravity
%       .m1 = cart mass
%       .m2 = pole mass
%       .l = pendulum length
% OUTPUTS:
%   dz = dz/dt = time derivative of state
%
%

%%%% Angular position, velocity, and acceleration of the pendulum
q = x(2,:);
dq = dx(2,:);
ddq = ddx(2,:);

%%%% Optimization will drive "eqns" to zero using a constraint
eqns = autoGen_cartPoleDynamics(ddx(1,:),  q,dq,ddq,  u,  p.m1, p.m2, p.g, p.l);

end