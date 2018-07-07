function output = dirCol5i(problem)
% output = dirCol5i(problem)
%
% Solves a continuous-time, single-phase trajectory optimization problem,
% with implicit second order dynamics.
%
% NOTATION:
%
%       continuous (path) input, listed in order:
%               t = [1, nTime] = time vector (col points)
%               x = [nx, nTime] = position at each col point
%               dx = [nx, nTime] = velocity at each col point
%               ddx = [nx, nTime] = acceleration at each col point
%               dddx = [nx, nTime] = jerk at each col point
%               ddddx = [nx, nTime] = snap at each col point
%               u = [nu, nTime] = control vector at each col point
%               du = [nu, nTime] = control rate vector at each col point
%
%       boundary (end-point) input, listed in order:
%               t0 = initial time
%               t1 = final time
%               x0 = x(t0) = initial position
%               x1 = x(t1) = final position
%               dx0 = dx(t0) = initial velocity
%               dx1 = dx(t1) = final velocity
%
% INPUT: "problem" -- struct with fields:
%
%   func -- struct for user-defined functions, passed as function handles
%
%       zero = dynamics(t,x,dx,ddx,u)
%               zero = [nx,nTime] = equality constraint, drive to zero
%
%       dObj = pathObj(t,x,dx,ddx,u,  dddx, ddddx, du)
%               dObj = [1, nTime] = integrand from the cost function
%
%       obj = bndObj(boundary)
%               obj = scalar = objective function for boundry points
%
%       [c, ceq] = pathCst(t,x,dx,ddx,u)
%               c = column vector of inequality constraints  ( c <= 0 )
%               ceq = column vector of equality constraints ( c == 0 )
%
%       [c, ceq] = bndCst(boundary)
%               c = column vector of inequality constraints  ( c <= 0 )
%               ceq = column vector of equality constraints ( c == 0 )
%
%       How to pass parameters to your functions:
%           - suppose that your dynamics function is pendulum.m and it
%           accepts a struct of parameters p. When you are setting up the
%           problem, define the struc p in your workspace and then use the
%           following command to pass the function:
%               problem.func.dynamics = @(t,x,u)( pendulum(t,x,u,p) );
%
%
%   bounds - struct with bounds for the problem:
%
%       .t0.low = [scalar]
%       .t0.upp = [scalar]
%
%       .t1.low = [scalar]
%       .t1.upp = [scalar]
%
%       .x0.low = [nx,1]   = bounds on initial position
%       .x0.upp = [nx,1]
%
%       .x.low = [nx,1]  = bound on position along the trajectory
%       .x.upp = [nx,1]
%
%       .x1.low = [nx,1]   = bounds on final position
%       .x1.upp = [nx,1]
%
%       .dx0.low = [nx,1]   = bounds on initial velocity
%       .dx0.upp = [nx,1]
%
%       .dx.low = [nx,1]  = bound on velocity along the trajectory
%       .dx.upp = [nx,1]
%
%       .dx1.low = [nx,1]   = bounds on final velocity
%       .dx1.upp = [nx,1]
%
%       .u.low = [nu, 1]   = bounds on control along the trajectory
%       .u.upp = [nu, 1]
%
%
%
%   guess - struct with an initial guess at the trajectory
%
%       .t = [1, nt]
%       .x = [nx, nt]
%       .dx = [nx, nt]
%       .ddx = [nx, nt]    (optional)
%       .u = [nu, nt]
%       .du = [nu, nt]    (optional)
%
%   options = options for the transcription algorithm (this function)
%
%       .nlpOpt = options to pass through to fmincon
%
%       .mesh = struct of options for constructing and refining the mesh
%           .nSegmentInit = scalar int = number of segment for first mesh
%           .tol = scalar = tolerance for mesh refinement in an interval
%           .maxIter = scalar int = limit on mesh refinement (1 = no refinement)
%           .maxSubDivide = scalar int = limit on how segment sub-division
%
%
% OUTPUT: "ouput"  --  struct with fields:
%
%   .options = copy of problem.options, includeding default values
%
%   .bounds = copy of problem.bounds;
%
%   .func = copy of problem.func
%
%   .soln(nIter) = struct array with information about each mesh iteration
%
%
%       .grid = trajectory at the collocation points
%           .t = [1, nCol] = time
%           .w = [1, nCol] = quadrature weights
%           .x = [nx, nCol] = position
%           .dx = [nx, nCol] = velocity
%           .ddx = [nx, nCol] = acceleration
%           .dddx = [nx, nCol] = jerk
%           .ddddx = [nx, nCol] = snap
%           .u = [nu, nCol] = control
%           .du = [nu, nCol] = control rate
%
%       .pp = A collection of "pp" structs that define piecewise-polynomial
%   splines in matlab. Evaluate the splines using ppval. The field names
%   are identical to those in col. For interpolating the solution. Splines
%   are method-consistent. Also, derivative relationships are obtained by
%   direct differentiation of the splines (in both the interpolation and
%   the collocation method).
%
%       .info = information about the optimization run
%           .nlpTime = time (seconds) spent in fmincon
%           .exitFlag = fmincon exit flag
%           .objVal = value of the objective function
%           .[all fields in the fmincon "output" struct]
%
%       .guess = initialization for each mesh iteration
%
%       .mesh = details about the mesh for this iteration
%
%

global DIM USE_LOBOTTO_POINTS

USE_LOBOTTO_POINTS = true;  % True == algorithm in paper

% Check the inputs, populate with defaults
problem = inputValidation(problem);

% To make code more readable
B = problem.bounds;
F = problem.func;
Opt = problem.options;

% Basic problem dimension stuff:
DIM = [];
DIM.nx = size(problem.guess.x,1);
DIM.nu = size(problem.guess.u,1);

% Convert the user's guess into a spline for interpolation:
G.x = spline(problem.guess.t, problem.guess.x);
G.dx = spline(problem.guess.t, problem.guess.dx);
G.ddx = spline(problem.guess.t, problem.guess.ddx);
G.u = spline(problem.guess.t, problem.guess.u);
G.du = spline(problem.guess.t, problem.guess.du);

% Compute an initial mesh to solve on:
nSegment = Opt.mesh.nSegmentInit;
G.frac = ones(1,nSegment)/nSegment;
G.tSpan = problem.guess.t([1,end]);

for iMesh = 1:Opt.mesh.maxIter
    
    if iMesh == 1
        soln(iMesh) = dirColGaussCore(G,B,F,Opt); %#ok<AGROW>
    else
        G = soln(iMesh-1).pp;
        G.frac = soln(iMesh-1).mesh.proposed;
        G.tSpan = soln(iMesh-1).knotPts.t([1,end]);
        soln(iMesh) = dirColGaussCore(G,B,F,Opt); %#ok<AGROW>
    end
    
    if soln(iMesh).mesh.converged
        break;
    end
    
end

output.soln = soln;
output.bounds = B;
output.func = F;
output.options = Opt;
output.problem = problem;

% General stats about the problem:
nMesh = length(soln);
output.summary.nlp.objVal = zeros(1,nMesh);
output.summary.nlp.time = zeros(1,nMesh);
output.summary.nlp.exit = zeros(1,nMesh);
output.summary.nlp.nIter = zeros(1,nMesh);
output.summary.nlp.nEval = zeros(1,nMesh);
output.summary.mesh.nSegment = zeros(1,nMesh);
output.summary.mesh.maxError = zeros(1,nMesh);
for i=1:nMesh
    output.summary.nlp.objVal(i) = output.soln(i).info.objVal;
    output.summary.nlp.time(i) = output.soln(i).info.nlpTime;
    output.summary.nlp.exit(i) = output.soln(i).info.exitFlag;
    output.summary.nlp.nIter(i) = output.soln(i).info.iterations;
    output.summary.nlp.nEval(i) = output.soln(i).info.funcCount;
    output.summary.mesh.nSegment(i) = length(output.soln(i).mesh.previous);
    output.summary.mesh.maxError(i) = output.soln(i).mesh.maxErr;
end

end


function soln = dirColGaussCore(G,B,F,Opt)
%
% This is where the actual work of the algorithm comes in
%

global DIM    %Store the problem dimensions

% Compute dimensions for the problem
unpackDecVars([]);  %Force a reset of the persistent variables.
computeDimensions(G.frac);    % sets DIM   (global variable)


%%%% Unpack problem bounds:
tLow = [B.t0.low, B.t1.low];
tUpp = [B.t0.upp, B.t1.upp];

xLow = [B.x0.low, B.x.low*ones(1,DIM.nKnot-2), B.x1.low];
xUpp = [B.x0.upp, B.x.upp*ones(1,DIM.nKnot-2), B.x1.upp];

dxLow = [B.dx0.low, B.dx.low*ones(1,DIM.nKnot-2), B.dx1.low];
dxUpp = [B.dx0.upp, B.dx.upp*ones(1,DIM.nKnot-2), B.dx1.upp];

ddxLow = -inf(size(dxLow));
ddxUpp = inf(size(dxUpp));

uLow = B.u.low*ones(1,DIM.nKnot);
uUpp = B.u.upp*ones(1,DIM.nKnot);

duLow = -inf(size(uLow));
duUpp = inf(size(uUpp));

zLow = packDecVars(tLow, xLow, dxLow, ddxLow, uLow, duLow);
zUpp = packDecVars(tUpp, xUpp, dxUpp, ddxUpp, uUpp, duUpp);

%%%% Unpack the problem guess

tKnotGuess = G.tSpan(1) + (G.tSpan(2)-G.tSpan(1))*DIM.tKnot;
tBndGuess = G.tSpan;

xGuess = ppval(G.x,tKnotGuess);
dxGuess = ppval(G.dx,tKnotGuess);
ddxGuess = ppval(G.ddx,tKnotGuess);
uGuess = ppval(G.u,tKnotGuess);
duGuess = ppval(G.du,tKnotGuess);

zGuess = packDecVars(tBndGuess,xGuess,dxGuess,ddxGuess,uGuess,duGuess);

%%%% Set-Up for fmincon
P.objective = @(z)( myObjective(z,F) );
P.nonlcon = @(z)( myConstraint(z,F,B) );
P.x0 = zGuess;
P.lb = zLow;
P.ub = zUpp;
P.Aineq = []; P.bineq = [];
P.Aeq = []; P.beq = [];
P.options = Opt.nlpOpt;
P.solver = 'fmincon';

%%%% Call fmincon to solve the non-linear program (NLP)
tic;
[zSoln, objVal,exitFlag,output] = fmincon(P);
nlpTime = toc;

% Unpack the solution:
[t, w, x, dx, ddx, dddx, ddddx, u, du,...
    soln.pp, soln.knotPts] = unpackDecVars(zSoln);

% Store the solution col:
soln.colPts = makeStruct(t, w, x, dx, ddx, dddx, ddddx, u, du);

% Initial guess:
soln.guess = G;

% Store the solver info
soln.info = output;
soln.info.nlpTime = nlpTime;
soln.info.exitFlag = exitFlag;
soln.info.objVal = objVal;

% Compute the error estimates:
soln.mesh = meshAnalysis(soln,F,Opt);

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function computeDimensions(frac)
%
% Extract relevant dimensions and other constants from the problem struct
%
global DIM

% Col Fraction = fraction of trajectory for each segment
DIM.meshFraction = frac;
DIM.meshFraction = DIM.meshFraction/sum(DIM.meshFraction);  %Normalize
DIM.nSegment = length(DIM.meshFraction);

%Time at knot points, with trajectory mapped to [0,1]
DIM.tKnot = cumsum([0,DIM.meshFraction]);
DIM.nKnot = length(DIM.tKnot);  %Number of knot points

% Time at the boundaries of each segment:
DIM.tLow = DIM.tKnot(1:(end-1));
DIM.tUpp = DIM.tKnot(2:end);

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function z = packDecVars(t,x,dx,ddx,u,du)
%
% Packs all decision variables into a single column vector
%
% INPUTS:
%   tBnd = [t0, t1] = [initialTime, finalTime]
%   xKnot = [nx, nt] = position at each knot point
%   dxKnot = [nx, nt] = velocity at each knot point
%   ddxKnot = [nx, nt] = acceleration at each knot point
%   uCol = [nu, nt] = control at lower and upper edge of each segment
%
% OUTPUTS:
%   z = decision variables = [tBnd;xKnot;vKnot;uCol], where each component
%   is a column vector, created using the reshape command.
%

global DIM   % Holds problem dimensions

t = t([1,end]);  %Allow passing entire time vector or just boundary

z = [...
    reshape(t,2,1);
    reshape(x,DIM.nx*DIM.nKnot,1);
    reshape(dx,DIM.nx*DIM.nKnot,1);
    reshape(ddx,DIM.nx*DIM.nKnot,1);
    reshape(u,DIM.nu*DIM.nKnot,1);
    reshape(du,DIM.nu*DIM.nKnot,1)];

end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function [...
    tCol, wCol, ...
    xCol, dxCol, ddxCol, dddxCol, ddddxCol,...
    uCol, duCol,...
    pp, knotPts] = unpackDecVars(z)
%
% Unpacks the decision variables into useful matricies
%
% INPUTS:
%   z = decision variables = [tBnd;xKnot;vKnot;uKnot;duKnot], where each
%   component is a column vector, created using the reshape command.
%
% OUTPUTS
%   tCol = [1, nc] = time at each col points
%   wCol = [1, nc] = quadrature weights
%   xCol = [nx, nc] = position at col points
%   dxCol = [nx, nc] = velocity at col points
%   ddxCol = [nx, nc] = accel at col points
%   dddxCol = [nx, nc] = jerk at col points
%   ddddxCol = [nx, nc] = snap at col points
%   uCol = [nu, nc] = control at col points
%   duCol = [nu, nc] = derivative of control at col points
%   pp = pp-splines for state, control, and derivatives
%

global DIM   % Holds problem dimensions

persistent zLast tLast wLast xLast dxLast ddxLast dddxLast ddddxLast uLast duLast

%%%% Figure out if we need to recompute the spline interpolants
if isempty(z)   %Used to force a reset of the persistent variables.
    zLast = [];
    return;
elseif isempty(zLast)
    zLast = zeros(size(z));
end
recompute = nargout > 9 || any(zLast~=z);
if recompute  % Then we will need to compute the spline interpolation
    zLast = z;
    
    % Compute indicies for extraction
    tIdx = 1:2;
    xIdx = tIdx(end) + (1:(DIM.nx*DIM.nKnot));
    dxIdx = xIdx(end) + (1:(DIM.nx*DIM.nKnot));
    ddxIdx = dxIdx(end) + (1:(DIM.nx*DIM.nKnot));
    uIdx = ddxIdx(end) + (1:(DIM.nu*DIM.nKnot));
    duIdx = uIdx(end) + (1:(DIM.nu*DIM.nKnot));
    
    % Extract and reshape to vectors
    tBnd = z(tIdx);
    t = tBnd(1) + (tBnd(2)-tBnd(1))*DIM.tKnot;
    x = reshape(z(xIdx), DIM.nx, DIM.nKnot);
    dx = reshape(z(dxIdx), DIM.nx, DIM.nKnot);
    ddx = reshape(z(ddxIdx), DIM.nx, DIM.nKnot);
    u = reshape(z(uIdx), DIM.nu, DIM.nKnot);
    du = reshape(z(duIdx), DIM.nu, DIM.nKnot);
    
    % Interpolate the values at the collocation points:
    [tLast, wLast, xLast, dxLast, ddxLast, dddxLast, ddddxLast] = getColPtState(t,x,dx,ddx);
    [~, uLast, duLast] = getColPtControl(t,u,du);
end

tCol = tLast;
wCol = wLast;
xCol = xLast;
dxCol = dxLast;
ddxCol = ddxLast;
dddxCol = dddxLast;
ddddxCol = ddddxLast;
uCol = uLast;
duCol = duLast;

if nargout > 9
    % This is only called on the very last iteration, to compute the
    % pp-splines for interpolating the final solution.
    [pp.x, pp.dx, pp.ddx, pp.dddx, pp.ddddx] = getPpState(t,x,dx,ddx);
    [pp.u, pp.du] = getPpControl(t,u,du);
    knotPts = makeStruct(t,x,dx,ddx, u, du);
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function cost = myObjective(z,F)
%
% This function unpacks the decision variables, sends them to the
% user-defined objective functions, and then returns the final cost
%
% INPUTS:
%   z = column vector of decision variables
%   pack = details about how to convert decision variables into t,x, and u
%   pathObj = user-defined integral objective function
%   endObj = user-defined end-point objective function
%
% OUTPUTS:
%   cost = scale cost for this set of decision variables
%

% Compute state and controls at collocation points
[t, w,x, dx, ddx, dddx, ddddx,u, du] = unpackDecVars(z);

% Compute the cost integral along trajectory
if isempty(F.pathObj)
    integralCost = 0;
else
    integrand = F.pathObj(t,x,dx,ddx,u,  dddx, ddddx, du);  %Calculate the integrand of the cost function
    integralCost = sum(integrand*w');
end

% Compute the cost at the boundaries of the trajectory
if isempty(F.bndObj)
    bndCost = 0;
else
    t0 = t(1);
    t1 = t(end);
    x0 = x(:,1);
    x1 = x(:,end);
    dx0 = dx(:,1);
    dx1 = dx(:,end);
    bndCost = F.bndObj(t0,t1,x0,x1,dx0,dx1);
end

cost = bndCost + integralCost;

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function [c, ceq] = myConstraint(z, F, B)
%
% This function unpacks the decision variables, computes the defects along
% the trajectory, and then evaluates the user-defined constraint functions.
%
% INPUTS:
%   z = column vector of decision variables
%   F = struct with user-defined functions
%   B = struct with bounds
%
% OUTPUTS:
%   c = inequality constraints to be passed to fmincon
%   ceq = equality constraints to be passed to fmincon
%

global USE_LOBOTTO_POINTS

%%%% Extract trajectory from decision variables
[t, ~,x, dx, ddx, ~, ~,u, ~] = unpackDecVars(z);

if USE_LOBOTTO_POINTS
    %%%% Remove redundant points:
    % The lobotto points are doubled on the intermediate segment endpoints
    idxRm = 4:4:(length(t)-1);
    t(idxRm) = [];
    x(:,idxRm) = [];
    dx(:,idxRm) = [];
    ddx(:,idxRm) = [];
    u(:,idxRm) = [];
end

%%%% Solve dynamics along the trajectory:
dynDefect = F.dynamics(t,x,dx,ddx,u);  %Implicit dynamics
nt = length(t);
nx = size(x,1);
ceq_dyn = reshape(dynDefect,nt*nx,1);

%%%% Compute the user-defined constraints:
if isempty(F.pathCst)
    c_path = [];
    ceq_path = [];
else
    [c_pathRaw, ceq_pathRaw] = F.pathCst(t,x,dx,ddx,u);
    c_path = reshape(c_pathRaw,numel(c_pathRaw),1);
    ceq_path = reshape(ceq_pathRaw,numel(ceq_pathRaw),1);
end
if isempty(F.bndCst)
    c_bnd = [];
    ceq_bnd = [];
else
    boundary.t0 = t(1);
    boundary.t1 = t(end);
    boundary.x0 = x(:,1);
    boundary.x1 = x(:,end);
    boundary.dx0 = dx(:,1);
    boundary.dx1 = dx(:,end);
    [c_bnd, ceq_bnd] = F.bndCst(boundary);
end

%%%% Enforce constraints on collocation points:
% TODO:  This code could be optimized much better
nc = length(t);  %Number of collocation points
One = ones(1,nc);  % used for vectorization (below)
c_collPtCst = [...
    B.x.low*One - x;
    x - B.x.upp*One;
    B.dx.low*One - dx;
    dx - B.dx.upp*One;
    B.u.low*One - u;
    u - B.u.upp*One];
c_collPtCst = c_collPtCst(~isinf(c_collPtCst));
c_collPtCst = reshape(c_collPtCst,numel(c_collPtCst),1);

%%%% Pack everything up:
c = [c_path; c_bnd; c_collPtCst];
ceq = [ceq_dyn; ceq_path; ceq_bnd];
end



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

function problem = inputValidation(user)
% problem = inputValidation(problem)
%
% Does input validation on the problem struct, as well as filling in
% default values for all fields.
%
% This is a utility function, and should not be called by the user.
%
% For now, this just fills in defaults, and does not do a careful check on
% the inputs.
%


%%%% Set up the default values:

[nx,nt] = size(user.guess.x);
nu = size(user.guess.u, 1);

default.func.dynamics = [];
default.func.pathObj = [];
default.func.bndObj = [];
default.func.pathCst = [];
default.func.bndCst = [];

default.bounds.t0.low = 0;
default.bounds.t0.upp = 0;

default.bounds.t1.low = 0;
default.bounds.t1.upp = inf;

default.bounds.x.low = -inf(nx,1);
default.bounds.x.upp = inf(nx,1);

default.bounds.dx.low = -inf(nx,1);
default.bounds.dx.upp = inf(nx,1);

default.bounds.u.low = -inf(nu,1);
default.bounds.u.upp = inf(nu,1);

default.bounds.du.low = -inf(nu,1);
default.bounds.du.upp = inf(nu,1);

default.bounds.x0.low = -inf(nx,1);
default.bounds.x0.upp = inf(nx,1);

default.bounds.dx0.low = -inf(nx,1);
default.bounds.dx0.upp = inf(nx,1);

default.bounds.x1.low = -inf(nx,1);
default.bounds.x1.upp = inf(nx,1);

default.bounds.dx1.low = -inf(nx,1);
default.bounds.dx1.upp = inf(nx,1);

default.guess.t = linspace(0,1,nt);
default.guess.x = zeros(nx,nt);
default.guess.dx = zeros(nx,nt);
default.guess.ddx = zeros(nx,nt);
default.guess.u = zeros(nu,nt);
default.guess.du = zeros(nu,nt);

default.options.nlpOpt = optimset('fmincon');
default.options.nlpOpt.Display = 'iter';
default.options.nlpOpt.MaxFunEvals = 5e4;

default.options.mesh.nSegmentInit = 5;
default.options.mesh.tol = 1e-3;
default.options.mesh.maxIter = 5;
default.options.mesh.maxSubDivide = 4;

default.auxdata = [];

%%%% Merge with the user-defined options:
problem = mergeOptions(default, user, 'problem');

%%%% Input Validation:

if isempty(problem.func.dynamics)
    error('User must specify:  problem.func.dynamics');
end

if problem.options.mesh.maxIter < 1
    warning('problem.options.mesh.maxIter = %d must be positive!',problem.options.mesh.maxIter);
    disp('  -->  setting:  problem.options.mesh.maxIter = 1;');
    problem.options.mesh.maxIter = 1;
end

end


