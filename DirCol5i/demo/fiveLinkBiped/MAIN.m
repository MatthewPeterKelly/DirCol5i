% MAIN.m  --  Five Link Biped trajectory optimization
%
% This script sets up and then solves the optimal trajectory for the five
% link biped, assuming that the walking gait is compused of single-stance
% phases of motion connected by impulsive heel-strike (no double-stance or
% flight phases).
%
% The equations of motion and gradients are all derived by:
%   --> Derive_Equations.m
%

clc; clear;
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up parameters and options                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
param = getPhysicalParameters();

param.stepLength = 0.5;
param.stepTime = 0.8;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics =  @(t,x,dx,ddx,u)( dynamics(t,x,dx,ddx,u,param) );

problem.func.pathObj = @(t,x,dx,ddx,u,dddx,ddddx,du)( obj_torqueSquared(u) );

problem.func.bndCst = @(boundary)( stepConstraint(boundary, param) );

problem.func.pathCst = @(t,x,dx,ddx,u)( pathConstraint(x) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.t0.low = 0;
problem.bounds.t0.upp = 0;
problem.bounds.t1.low = param.stepTime;
problem.bounds.t1.upp = param.stepTime;

% State: (absolute reference frames)
%   1 = stance leg tibia angle
%   2 = stance leg femur angle
%   3 = torso angle
%   4 = swing leg femur angle
%   5 = swing leg tibia angle

qLow = (-pi/2)*ones(5,1);
qUpp = (pi/2)*ones(5,1);
dqLow = -10*ones(5,1);
dqUpp = 10*ones(5,1);

problem.bounds.x.low = qLow;
problem.bounds.x.upp = qUpp;
problem.bounds.x0.low = qLow;
problem.bounds.x0.upp = qUpp;
problem.bounds.x1.low = qLow;
problem.bounds.x1.upp = qUpp;

problem.bounds.dx.low = dqLow;
problem.bounds.dx.upp = dqUpp;
problem.bounds.dx0.low = dqLow;
problem.bounds.dx0.upp = dqUpp;
problem.bounds.dx1.low = dqLow;
problem.bounds.dx1.upp = dqUpp;

uMax = 100;  %Nm
problem.bounds.u.low = -uMax*ones(5,1);
problem.bounds.u.upp = uMax*ones(5,1);

% Disable the stance ankle motor:
problem.bounds.u.low(1) = 0;
problem.bounds.u.upp(1) = 0;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values

problem.guess.t = [0, param.stepTime];

q0 = [...
    -0.3; % stance leg tibia angle
    0.7; % stance leg femur angle
    0.0; % torso angle
    -0.5; % swing leg femur angle
    -0.6]; % swing leg tibia angle
qF = q0([5;4;3;2;1]);   %Flip left-right

dq0 = (qF-q0)/param.stepTime;
dqF = dq0;

problem.guess.x = [q0, qF];
problem.guess.dx = [dq0, dqF];

problem.guess.u = zeros(5,2);  %Start with passive trajectory


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Method-independent options:
problem.options.nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-6,...
    'MaxFunEvals',1e5);   %options for fmincon

problem.options.mesh.nSegmentInit = 8;
problem.options.mesh.tol = 1e-4;
problem.options.mesh.maxIter = 3;
problem.options.mesh.maxSubDivide = 4;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%%% THE KEY LINE:
soln = dirCol5i(problem);

% Transcription Knot points:
tKnot = soln.soln(end).knotPts.t;
qKnot = soln.soln(end).knotPts.x;
dqKnot = soln.soln(end).knotPts.dx;
uKnot = soln.soln(end).knotPts.u;

% Transcription Collocation points:
tColl = soln.soln(end).colPts.t;
qColl = soln.soln(end).colPts.x;
dqColl = soln.soln(end).colPts.dx;
uColl = soln.soln(end).colPts.u;

% Interpolation
t = linspace(tKnot(1), tKnot(end), 250);
q = ppval(soln.soln(end).pp.x, t);
dq = ppval(soln.soln(end).pp.dx, t);
u = ppval(soln.soln(end).pp.u, t);


%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(1); clf;
h1 = subplot(1,2,1); hold on;
plot(t, q, 'LineWidth', 3);
plot(tKnot, qKnot, 'ks', 'MarkerSize', 6);
plot(tColl, qColl, 'kx', 'MarkerSize', 4);
legend('q1','q2','q3','q4','q5');
xlabel('time')
ylabel('link angles')

h2 = subplot(1,2,2); hold on;
plot(t, u, 'LineWidth', 3);
plot(tKnot, uKnot, 'ko', 'MarkerSize', 3);
plot(tColl, uColl, 'ks', 'MarkerSize', 2);
legend('u1','u2','u3','u4','u5');
xlabel('time')
ylabel('joint torques')

linkaxes([h1, h2], 'x');
