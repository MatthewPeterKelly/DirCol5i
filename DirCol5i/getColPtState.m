function [tCol, wCol, xCol, dxCol, ddxCol, dddxCol, ddddxCol] = getColPtState(t,x,dx,ddx)
% [tCol, wCol, xCol, dxCol, ddxCol, dddxCol, ddddxCol] = getColPtState(t,x,dx,ddx)
%
% This function computes the state and derivatives at the collocation
% points, given the state and derivatives at the knot points
%
% INPUTS:
%   t = [1,nt] = time at the knot points
%   x = [nx,nt] = position at the knot points
%   dx = [nx,nt] = velocity at the knot points
%   ddx = [nx,nt] = acceleration at the knot points
%
% OUTPUTS:   nc = 3*(nt-1)
%   tCol = [1, nc] = time at the collocation points
%   wCol = [1, nc] = quadrature weights
%   xCol = [nx, nc] = position at the collocation points
%   dxCol = [nx, nc] = velocity at the collocation points
%   ddxCol = [nx, nc] = acceleration at the collocation points
%
% NOTES:
%   The collocation points here are computed using Gauss-Legendre points.
%   These three points per segment give 5th-order quadrature rules.
%

%%%% Dimension stuff:
ng = 4;  %Number of gauss points on each segment
nt = length(t);
ns = nt-1;   %number of segments
nc = ng*ns;  % total number of collocation points
nx = size(x,1);


%%%% Break apart by segment:
iA = 1:(nt-1);
iB = 2:nt;
tA = t(iA);
tB = t(iB);
duration = tB-tA;  %duration of each interval
midpoint = 0.5*(tA+tB);   %midpoint of each interval
scale1 = 0.5*ones(nx,1)*duration;  %matrix domain scaling term
scale2 = scale1.*scale1;
invScale1 = 1./scale1;
invScale2 = invScale1.*invScale1;
invScale3 = invScale1.*invScale2;
invScale4 = invScale1.*invScale3;

xA = x(:,iA);
dxA = scale1.*dx(:,iA);
ddxA = scale2.*ddx(:,iA);

xB = x(:,iB);
dxB = scale1.*dx(:,iB);
ddxB = scale2.*ddx(:,iB);

%%%% Compute mapping on normalized intervals
[tColPts, wColPts] = computeGaussPoints();
tCol = zeros(1,nc);
wCol = zeros(1,nc);
Midpoint = zeros(1,nc);
Duration = zeros(1,nc);
Scale1 = zeros(nx,nc);
Scale2 = zeros(nx,nc);
Scale3 = zeros(nx,nc);
Scale4 = zeros(nx,nc);
xCol = zeros(nx,nc);
dxCol = zeros(nx,nc);
ddxCol = zeros(nx,nc);
dddxCol = zeros(nx,nc);
ddddxCol = zeros(nx,nc);
idx = 1:ng:nc;
for i=1:ng  %Loop over collocation points
    index = (i-1)+(idx);
    Midpoint(index) = midpoint;
    Duration(index) = duration;
    Scale1(:,index) = invScale1;
    Scale2(:,index) = invScale2;
    Scale3(:,index) = invScale3;
    Scale4(:,index) = invScale4;
    tCol(index) = tColPts(i);
    wCol(index) = wColPts(i);
[xCol(:,index), dxCol(:,index),ddxCol(:,index),...  
    dddxCol(:,index),ddddxCol(:,index)] = ...
    autoGen_getColPtState(tColPts(i),...   Interpolation
    xA,xB,dxA,dxB,ddxA,ddxB);
end

%%%% Map back to original interval:
tCol = Midpoint + 0.5*Duration.*tCol;   %Collocation points
wCol = 0.5*Duration.*wCol;  %Quadrature weights
dxCol = dxCol.*Scale1;
ddxCol = ddxCol.*Scale2;
dddxCol = dddxCol.*Scale3;
ddddxCol = ddddxCol.*Scale4;

end



