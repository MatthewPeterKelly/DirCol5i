function mesh = meshAnalysis(soln,F,Opt)
%
% This function computes the error estimates for a given solution to the
% trajectory optimization problem.
%

t = soln.knotPts.t;

ns = length(t)-1;
nx = size(soln.knotPts.x,1);

Eta = zeros(nx,ns);  % Integral of absolute error in each segment
dynErr = @(time)( abs(getDynErr(time,soln,F)) );  % Error at time t in the dynamics

tol = Opt.mesh.tol/10;  %Error estimate tolerance (slightly overestimate)

for k=1:ns  % Loop through each segment
Eta(:,k) = rombergQuadrature(dynErr, t([k,k+1]), tol); 
end

mesh = meshAnalysisCore(Eta,Opt,soln);
mesh.dynErrFun = dynErr;   % pp struct to interpolate error in each segment


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dynErr = getDynErr(t,soln,F)
%
% Compute the local defect in the dynamics equations
%

x = ppval(soln.pp.x,t);
dx = ppval(soln.pp.dx,t);
ddx = ppval(soln.pp.ddx,t);
u = ppval(soln.pp.u,t);

dynErr = F.dynamics(t,x,dx,ddx,u);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function mesh = meshAnalysisCore(Eta,Opt,soln)

tol = Opt.mesh.tol;
nMax = Opt.mesh.maxSubDivide;

err = max(Eta,[],1);  %Choose dimension with maximum error in each segment

methodOrder = 6;  % Collocation method order

tolTarget = 0.1*tol;  % From Betts - try to do about 10 times better than required
n = (tolTarget./err).^(-1/methodOrder);  % Estimate how many sub-divisions are required
n = ceil(n);
n(n<1) = 1;
n(n>nMax) = nMax;

mesh.tol = tol;
mesh.maxErr = max(max(Eta)); 
mesh.converged = mesh.maxErr < mesh.tol;
mesh.segErr = Eta;   %Integral of absolute error in each segment
mesh.subdivide = n;
mesh.previous = soln.guess.frac;
mesh.proposed = getNewMesh(mesh.previous, n);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function newMesh = getNewMesh(oldMesh, nSubDivide)
%
% Sub-divides the old mesh to get the new mesh.
%
 
newMesh = zeros(1,sum(nSubDivide));

idx = 0;
for k = 1:length(nSubDivide)
    index = idx + (1:nSubDivide(k));
    newMesh(index) = ones(1,nSubDivide(k))*oldMesh(k)/nSubDivide(k);
    idx = index(end);
end

end





