function zero = dynamics(t,x,dx,ddx,u,p)
% zero = dynamics(t,x,dx,ddx,u,p)
%
% Computes the second-order implicit dynamics of the five-link biped
%
% INPUTS:
%
%  TODO:  finish documentation
%
%

nt = length(t);
empty = zeros(1,nt);  % For vectorization
q = x;
dq = dx;
ddqDyn = zeros(size(q));

[m,mi,f,fi] = autoGen_dynSs(...
    q(1,:),q(2,:),q(3,:),q(4,:),q(5,:),...
    dq(1,:),dq(2,:),dq(3,:),dq(4,:),dq(5,:),...
    u(1,:),u(2,:),u(3,:),u(4,:),u(5,:),...
    p.m1, p.m2, p.m3, p.m4, p.m5, p.I1, p.I2, p.I3, p.I4, p.I5, ...
    p.l1, p.l2, p.l3, p.l4, p.c1, p.c2, p.c3, p.c4, p.c5, p.g, empty);

M = zeros(5,5);  %Mass matrix
F = zeros(5,1);
for i=1:nt
    M(mi) = m(:,i);
    F(fi) = f(:,i);
    ddqDyn(:,i) = M\F;  %Numerically invert the mass matrix
end

zero = ddx - ddqDyn;

end
