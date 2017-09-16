function fraction = computeNewMesh(soln)
% fraction = computeNewMesh(soln)
%
% This function computes a new mesh by dividing the worst segments of the
% trajectory into two segments. Segments are ranked by the violation in the
% implicit dynamics at the mid-point.
%

%%%% Compute the mid-point of each segment
N = length(soln.grid.t);
idxLow = 1:2:N;
idxUpp = 2:2:N;
nSegment = length(idxLow);
tLow = soln.grid.t(idxLow);
tUpp = soln.grid.t(idxUpp);
t = 0.5*(tLow + tUpp);

%%%% Evaluate the trajectory at the mid-points:
x = soln.interp.x(t);
dx = soln.interp.dx(t);
ddx = soln.interp.ddx(t);
dddx = soln.interp.dddx(t);
u = soln.interp.u(t);
pathInput = makeStruct(t,x,dx,ddx,dddx,u);

%%%% Compute the error in the collocation constraint (dynamics) at the
%    mid-point of each segment of the trajectory
dynErr = soln.problem.func.dynamics(pathInput);

% Compute a normalized error estimate for each segment:
error = zeros(1,nSegment);
nx = size(dynErr,1);
for i=1:nx
    err = abs(dynErr(i,:));
   error = error + (err./max(err)).^2;
end
error = sqrt(error);
meanError = mean(error);
flagDivide = error > meanError;

%%%% Compute the suggested next mesh by subdividing any intervals in which
%    errorFlag is true.
nDivide = sum(flagDivide);
fraction = zeros(1,nSegment+nDivide);
nSubSection = 3;
idx = 0;
for i=1:nSegment
    fOld = soln.problem.options.mesh.fraction(i);
    if flagDivide(i)
        for j=1:nSubSection
        idx = idx + 1; fraction(idx) = fOld/nSubSection;
        end
    else  % Keep the old mesh fraction here
        idx = idx + 1; fraction(idx) = fOld;
    end
end

end