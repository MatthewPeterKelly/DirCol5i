function output = wrapOptimTraj(problem)
% output = wrapOptimTraj(problem)
%
% Solves a continuous-time, single-phase trajectory optimization problem,
% with implicit second order dynamics.
%
% Converts problem from the form described here into a form that can be
% solved using standard methods (through OptimTraj)
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
%           .maxIter = scalar int = limit on mesh refinement (0 = no refinement)
%           .maxSubDivide = scalar int = limit on how segment sub-division
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


% TODO:
%
% Make a wrapper so that any problem that is formatted for dirColL4 can be
% solved by OptimTraj, using lots of extra states to handle the high
% derivatives.


end