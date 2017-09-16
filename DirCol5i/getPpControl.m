function [PPu, PPdu] = getPpControl(t,u,du)
% [PPu, PPdu] = getPpControl(t,u,du)
%
% This function computes pp-form splines for the control and control rate,
% to be evalauted by Matlab's ppval() command.
%
% INPUTS:
%   t = [1,nt] = time at the knot points
%   u = [nu,nt] = position at the knot points
%   du = [nu,nt] = velocity at the knot points
%
% OUTPUTS:   nc = 3*(nt-1)
%   PPu = pp-form struct with u trajectory
%   PPdu = pp-form struct with du trajectory
%

%%%% Dimension stuff:
nt = length(t);
nu = size(u,1);
ns = nt - 1;

%%%% Break apart by segment:
iA = 1:(nt-1);
iB = 2:nt;

tA = t(iA);
tB = t(iB);

%%%% Construct the state splines
tLow = ones(nu,1)*tA;
tUpp = ones(nu,1)*tB;
uLow = u(:,iA);
uUpp = u(:,iB);
duLow = du(:,iA);
duUpp = du(:,iB);
[C1,C2,C3,C4] = autoGen_controlSplineCoeffs(...
    0*tLow,tUpp-tLow,...
    uLow,uUpp,...
    duLow,duUpp);

% State:
orderCubic = 4;
C = zeros(nu*ns,orderCubic);
for i=1:ns
    idu = nu*(i-1)+(1:nu);
    C(idu,:) = [C4(:,i),C3(:,i),C2(:,i),C1(:,i)];
end
PPu = getPp(t,C,orderCubic,nu);

% Derivative:
C = zeros(nu*ns,orderCubic-1);
for i=1:ns
    idu = nu*(i-1)+(1:nu);
    C(idu,:) = [3*C4(:,i),2*C3(:,i),1*C2(:,i)];
end
PPdu = getPp(t,C,orderCubic-1,nu);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pp = getPp(tKnot,coeff,order,nDim)

pp.form = 'pp';
pp.breaks = tKnot;
pp.coefs = coeff;
pp.pieces = length(tKnot)-1;
pp.order = order;
pp.dim = nDim;

end


