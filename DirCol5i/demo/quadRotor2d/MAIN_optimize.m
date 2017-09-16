% MAIN  --  Quad-Rotor  --  Minimal-Force trajectory
%
% Fin the minimal torque-squared trajectory to move the quad-rotor from an
% arbitrary state to the origin.
%

clc; clear;
addpath ../../

% Dynamics paramters
p.g = 9.81; % (m/s^2) gravity
p.d = 0.3;  % (m) half-width of quad rotor
p.m = 0.2;  % (m) half-mass of the quad rotor

% Trajectory Parameters:
duration = 1;
uMax = 5*p.g*p.m;

% Initial State:
x0 = 1.0;
y0 = 0.0;
q0 = 0.0;
dx0 = 0.0;
dy0 = 0.0;
dq0 = 0.0;
z0 = [x0;y0;q0];
dz0 = [dx0; dy0;dq0];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,z,dz,ddz,u)( dynamics2i(z, ddz, u, p) );

% objMode = 'force';
% objMode = 'forceRate';
% objMode = 'accel';
% objMode = 'jerk';
objMode = 'snap';
% objMode = 'smooth';
switch objMode
    case 'force'   %Minimize the integral of force squared
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)( sum(u.^2,1) );
    case 'forceRate'   %Minimize the integral of force squared
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)( sum(du.^2,1));
    case 'accel'
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)( sum(ddx.^2,1) );
    case 'jerk'
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)( sum(dddx.^2,1) );
    case 'snap'
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)( sum(ddddx.^2,1) );
    case 'smooth'        
        problem.func.pathObj = ...
            @(t,x,dx,ddx,u,  dddx, ddddx, du)(  sum(du.^2,1) + 0.1*sum(dddx.^2,1) );
    otherwise
        error('Unrecognized Objective Function');
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.t0.low = 0;
problem.bounds.t0.upp = 0;
problem.bounds.t1.low = duration;
problem.bounds.t1.upp = duration;

problem.bounds.x0.low = z0;
problem.bounds.x0.upp = z0;
problem.bounds.x1.low = zeros(3,1);
problem.bounds.x1.upp = zeros(3,1);

problem.bounds.dx0.low = dz0;
problem.bounds.dx0.upp = dz0;
problem.bounds.dx1.low = zeros(3,1);
problem.bounds.dx1.upp = zeros(3,1);

problem.bounds.u.low = -uMax*[1;1];
problem.bounds.u.upp = uMax*[1;1];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.guess.t = [0,duration];
problem.guess.x = [z0, zeros(3,1)];
problem.guess.dx = [dz0, zeros(3,1)];
problem.guess.u = p.g*p.m*ones(2,2);



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

nSegment = 2;
problem.options.mesh.nSegmentInit = nSegment;
problem.options.mesh.maxIter = 2;
problem.options.mesh.tol = 1e-3;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

output = dirCol5i(problem);
soln = output.soln(end);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%%%% Unpack the simulation
t = linspace(soln.knotPts.t(1), soln.knotPts.t(end), 150);

z = ppval(soln.pp.x,t);
x = z(1,:);
y = z(2,:);
q = z(3,:);
dz = ppval(soln.pp.dx,t);
dx = dz(1,:);
dy = dz(2,:);
dq = dz(3,:);

u = ppval(soln.pp.u,t);
u1 = u(1,:);
u2 = u(2,:);

tCol = soln.colPts.t;
zCol = soln.colPts.x;
uCol = soln.colPts.u;
xCol = zCol(1,:);
yCol = zCol(2,:);
qCol = zCol(3,:);
u1Col = uCol(1,:);
u2Col = uCol(2,:);

%%%% Plots:
figure(1); clf;
color = lines();

subplot(2,2,1); hold on;
plot(t,x);
plot(tCol, xCol, 'ko' ,'MarkerSize',10, 'LineWidth',2);
xlabel('t')
ylabel('x')
title('Minimum force-squared trajectory')

subplot(2,2,2); hold on;
plot(t,y);
plot(tCol, yCol, 'ko' ,'MarkerSize',10, 'LineWidth',2);
xlabel('t')
ylabel('y')

subplot(2,2,3); hold on;
plot(t,q);
plot(tCol, qCol, 'ko' ,'MarkerSize',10, 'LineWidth',2);
xlabel('t')
ylabel('q')

subplot(2,2,4); hold on;
plot(t,u1,'Color',color(1,:));
plot(t,u2,'Color',color(2,:));
plot(tCol,u1Col,'o','Color',color(1,:),'MarkerSize',10, 'LineWidth',2);
plot(tCol,u2Col,'o','Color',color(2,:),'MarkerSize',10, 'LineWidth',2);
xlabel('t')
ylabel('u')
legend('u1','u2');


