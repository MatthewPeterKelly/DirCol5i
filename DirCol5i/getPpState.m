function [PPx, PPdx, PPddx, PPdddx, PPddddx] = getPpState(t,x,dx,ddx)
% [PPx, PPdx, PPddx, PPdddx, PPddddx] = getPpState(t,x,dx,ddx)
%
% This function computes pp-form splines for the state and derivatives, to
% be evalauted by Matlab's ppval() command.
%
% INPUTS:
%   t = [1,nt] = time at the knot points
%   x = [nx,nt] = position at the knot points
%   dx = [nx,nt] = velocity at the knot points
%   ddx = [nx,nt] = acceleration at the knot points
%
% OUTPUTS:   nc = 3*(nt-1)
%   PPx = pp-form struct with x trajectory
%   PPdx = pp-form struct with dx trajectory
%   PPddx = pp-form struct with ddx trajectory
%   PPdddx = pp-form struct with dddx trajectory
%   PPddddx = pp-form struct with ddddx trajectory
%

%%%% Dimension stuff:
nt = length(t);
nx = size(x,1);
ns = nt - 1;

%%%% Break apart by segment:
iA = 1:(nt-1);
iB = 2:nt;

tA = t(iA);
tB = t(iB);

%%%% Construct the state splines
tLow = ones(nx,1)*tA;
tUpp = ones(nx,1)*tB;
xLow = x(:,iA);
xUpp = x(:,iB);
dxLow = dx(:,iA);
dxUpp = dx(:,iB);
ddxLow = ddx(:,iA);
ddxUpp = ddx(:,iB);
[C1,C2,C3,C4,C5,C6] = autoGen_stateSplineCoeffs(...
    0*tLow,tUpp-tLow,...
    xLow,xUpp,...
    dxLow,dxUpp,...
    ddxLow,ddxUpp);

% State:
orderQuintic = 6;
C = zeros(nx*ns,orderQuintic);
for i=1:ns
    idx = nx*(i-1)+(1:nx);
    C(idx,:) = [C6(:,i),C5(:,i),C4(:,i),C3(:,i),C2(:,i),C1(:,i)];
end
PPx = getPp(t,C,orderQuintic,nx);

% Derivative:
C = zeros(nx*ns,orderQuintic-1);
for i=1:ns
    idx = nx*(i-1)+(1:nx);
    C(idx,:) = [5*C6(:,i),4*C5(:,i),3*C4(:,i),2*C3(:,i),1*C2(:,i)];
end
PPdx = getPp(t,C,orderQuintic-1,nx);

% Second Derivative:
C = zeros(nx*ns,orderQuintic-2);
for i=1:ns
    idx = nx*(i-1)+(1:nx);
    C(idx,:) = [5*4*C6(:,i),4*3*C5(:,i),3*2*C4(:,i),2*1*C3(:,i)];
end
PPddx = getPp(t,C,orderQuintic-2,nx);

% Third Derivative:
C = zeros(nx*ns,orderQuintic-3);
for i=1:ns
    idx = nx*(i-1)+(1:nx);
    C(idx,:) = [5*4*3*C6(:,i),4*3*2*C5(:,i),3*2*1*C4(:,i)];
end
PPdddx = getPp(t,C,orderQuintic-3,nx);

% Fourth Derivative:
C = zeros(nx*ns,orderQuintic-4);
for i=1:ns
    idx = nx*(i-1)+(1:nx);
    C(idx,:) = [5*4*3*2*C6(:,i),4*3*2*1*C5(:,i)];
end
PPddddx = getPp(t,C,orderQuintic-4,nx);

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


