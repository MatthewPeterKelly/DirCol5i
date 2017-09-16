function [tCol, uCol, duCol] = getColPtControl(t,u,du)
% [tCol, uCol, duCol] = getColPtControl(t,u,du)
%
% This function computes the state and derivatives at the collocation
% points, given the control and derivatives at the knot points
%
% INPUTS:
%   t = [1,nt] = time at the knot points
%   u = [nu,nt] = position at the knot points
%   du = [nu,nt] = velocity at the knot points
%
% OUTPUTS:   nc = 3*(nt-1)
%   tCol = [1, nc] = time at the collocation points
%   uCol = [nx, nc] = position at the collocation points
%   duCol = [nx, nc] = velocity at the collocation points
%
% NOTES:
%   The collocation points here are computed using Gauss-Legendre points.
%   These three points per segment give 5th-order quadrature rules.
%

%%%% Dimension stuff:
ng = 4;  %Number of gauss points on each segment
nt = length(t);
ns = nt-1;
nc = ng*ns; 
nu = size(u,1);

%%%% Break apart by segment:
iA = 1:(nt-1);
iB = 2:nt;

tA = t(iA);
tB = t(iB);
duration = tB-tA;  %duration of each interval
midpoint = 0.5*(tA+tB);   %midpoint of each interval
scale = 0.5*ones(nu,1)*duration;  %matrix domain scaling term
invScale = 1./scale;

uA = u(:,iA);
duA = scale.*du(:,iA);

uB = u(:,iB);
duB = scale.*du(:,iB);

%%%% Compute mapping on normalized intervals
tColPts = computeGaussPoints();
tCol = zeros(1,nc);
Midpoint = zeros(1,nc);
Duration = zeros(1,nc);
Scale = zeros(nu,nc);
uCol = zeros(nu,nc);
duCol = zeros(nu,nc);
idx = 1:ng:nc;
for i=1:ng  %Loop over collocation points
    index = (i-1)+(idx);
    Midpoint(index) = midpoint;
    Duration(index) = duration;
    Scale(:,index) = invScale;
    tCol(index) = tColPts(i);
[uCol(:,index), duCol(:,index)] = autoGen_getColPtControl(tColPts(i),uA,uB,duA,duB);
end

%%%% Map back to original interval:
tCol = Midpoint + 0.5*Duration.*tCol;
duCol = duCol.*Scale;

end



