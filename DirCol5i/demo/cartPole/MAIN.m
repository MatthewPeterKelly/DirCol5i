% MAIN.m
%
% Solve the cart-pole swing-up problem

clc; clear;
addpath ../../

% Dynamics paramters
p.m1 = 2.0;  % (kg) cart mass
p.m2 = 0.5;  % (kg) pole mass
p.g = 9.81;  % (m/s^2) gravity
p.l = 0.5;   % (m) pendulum (pole) length

% Trajectory Parameters:
distance = 1.0;  %How far must the cart translate during its swing-up
maxForce = 20;  %Maximum actuator forces
duration = 2;


problem.auxdata = makeStruct(distance,maxForce,duration);
problem.auxdata.dynParam = p;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,dx,ddx,u)( cartPoleDynamics(x,dx,ddx,u,p) );

objMode = 'force'; %          <--
% objMode = 'forceRate';
% objMode = 'accel';
% objMode = 'posAccel';
% objMode = 'jerk';
% objMode = 'posJerk';
% objMode = 'snap';
% objMode = 'posSnap'; %        <--
% objMode = 'smooth';  %        <--
% objMode = 'verySmooth';

objFun.force = @(t,x,dx,ddx,u,dddx,ddddx,du)( 0.01*u.^2 );
objFun.forceRate = @(t,x,dx,ddx,u,dddx,ddddx,du)( 0.001*du.^2 );
objFun.accel = @(t,x,dx,ddx,u,dddx,ddddx,du)( 0.01*sum(ddx.^2,1) );
objFun.posAccel = @(t,x,dx,ddx,u,dddx,ddddx,du)( objAccel(x,dx,ddx,p) );
objFun.jerk = @(t,x,dx,ddx,u,dddx,ddddx,du)( 1e-4*sum(dddx.^2,1) );
objFun.posJerk = @(t,x,dx,ddx,u,dddx,ddddx,du)( objJerk(x,dx,ddx,dddx,p) );
objFun.snap = @(t,x,dx,ddx,u,dddx,ddddx,du)( 1e-6*sum(ddddx.^2,1) );
objFun.posSnap = @(t,x,dx,ddx,u,dddx,ddddx,du)( objSnap(x,dx,ddx,dddx,ddddx,p) );
objFun.smooth = @(t,x,dx,ddx,u,dddx,ddddx,du)(  0.001*du.^2 + 0.01*u.^2 );
objFun.verySmooth = @(t,x,dx,ddx,u,dddx,ddddx,du)( ...
    objSnap(x,dx,ddx,dddx,ddddx,p) + 0.001*du.^2 + 0.01*u.^2 );

%%%% Set the objective function
problem.func.pathObj = objFun.(objMode);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

x0 = zeros(2,1);
x1 = [distance;pi];

problem.bounds.t0.low = 0;
problem.bounds.t0.upp = 0;
problem.bounds.t1.low = duration;
problem.bounds.t1.upp = duration;

problem.bounds.x0.low = x0;
problem.bounds.x0.upp = x0;
problem.bounds.x1.low = x1;
problem.bounds.x1.upp = x1;

problem.bounds.dx0.low = zeros(2,1);
problem.bounds.dx0.upp = zeros(2,1);
problem.bounds.dx1.low = zeros(2,1);
problem.bounds.dx1.upp = zeros(2,1);

problem.bounds.x.low = [-2*distance;-2*pi];
problem.bounds.x.upp = [2*distance;2*pi];

problem.bounds.u.low = -maxForce;
problem.bounds.u.upp = maxForce;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.t = [0,duration];
problem.guess.x = [x0, x1];
problem.guess.dx = ((x1-x0)/duration)*[1,1];
problem.guess.u = [0,0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.mesh.nSegmentInit = 2;
problem.options.mesh.tol = 1e-4;
problem.options.mesh.maxIter = 3;
problem.options.mesh.maxSubDivide = 5;

output = dirCol5i(problem);


%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Plot Optimal trajectory against time:
figure(1523); clf;
plotSolution(output);


%%%% Draw Trajectory:
figure(6317); clf;
drawTrajectory(output);


%%%% Draw and Plot Trajectory:
figure(6318); clf;
drawPlotTrajectory(output);
