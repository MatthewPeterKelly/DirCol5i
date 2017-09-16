% MAIN  --  Simple pendulum swing up.

addpath ../..

clc; clear;

%%%% Specify boundary conditions
t0 = 0;
tF = 5;    

maxTorque = 1.0;
x0 = 0;
xF = pi;
dx0 = 0;
dxF = 0;

param.k = 1.0;  % gravity torque constant for pendulum model
param.b = 0.1;  % viscous damping constant

%%%% Function Handles:
problem.func.dynamics = @(t,x,dx,ddx,u)(  pendulum(x,dx,ddx,u)  );

objective = 'minJerk';
switch objective
    case 'minAccel'
            problem.func.pathObj = @(t,x,dx,ddx,u,dddx,ddddx,du)( ddx.^2 );
    case 'minJerk'
            problem.func.pathObj = @(t,x,dx,ddx,u,dddx,ddddx,du)( dddx.^2 );
    case 'minSnap'
            problem.func.pathObj = @(t,x,dx,ddx,u,dddx,ddddx,du)( ddddx.^2 );
    otherwise
        error('unrecognized objective');
end

%%%% Boundary conditions:
problem.bounds.t0.low = t0;
problem.bounds.t0.upp = t0;
problem.bounds.t1.low = tF;
problem.bounds.t1.upp = tF;
problem.bounds.x0.low = x0;
problem.bounds.x0.upp = x0;
problem.bounds.x1.low = xF;
problem.bounds.x1.upp = xF;
problem.bounds.dx0.low = dx0;
problem.bounds.dx0.upp = dx0;
problem.bounds.dx1.low = dxF;
problem.bounds.dx1.upp = dxF;
problem.bounds.u.low = -maxTorque;
problem.bounds.u.upp = maxTorque;

%%%% Initial Guess
problem.guess.t = [t0, tF];
problem.guess.x = [x0, xF];
problem.guess.dx = [1,1]*(tF-t0)/(xF-x0);
problem.guess.u = [0,0];

%%%% Options
problem.options.mesh.tol = 1e-3;

%%%% Solve!
output = dirCol5i(problem);

%% %% Unpack the solution:
tGrid = output.soln(end).colPts.t;
xGrid = output.soln(end).colPts.x;
dxGrid = output.soln(end).colPts.dx;
ddxGrid = output.soln(end).colPts.ddx;
dddxGrid = output.soln(end).colPts.dddx;
ddddxGrid = output.soln(end).colPts.ddddx;
uGrid = output.soln(end).colPts.u;
duGrid = output.soln(end).colPts.du;

t = linspace(tGrid(1), tGrid(end), 200);
x = ppval(output.soln(end).pp.x,t);
dx = ppval(output.soln(end).pp.dx,t);
ddx = ppval(output.soln(end).pp.ddx,t);
dddx = ppval(output.soln(end).pp.dddx,t);
ddddx = ppval(output.soln(end).pp.ddddx,t);
u = ppval(output.soln(end).pp.u,t);
du = ppval(output.soln(end).pp.du,t);

%% %% Plot the trajectory against time
figure(10); clf;

subplot(3,2,1); hold on;
plot(t,x)
plot(tGrid,xGrid,'ko','MarkerSize',8,'LineWidth',2);
title('position (angle)')

subplot(3,2,3); hold on;
plot(t,dx)
plot(tGrid,dxGrid,'ko','MarkerSize',8,'LineWidth',2);
title('velocity (angular rate)')

subplot(3,2,5); hold on;
plot(t([1,end]),[1,1]*maxTorque,'k--','LineWidth',1);
plot(t([1,end]),-[1,1]*maxTorque,'k--','LineWidth',1);
plot(t,u)
plot(tGrid,uGrid,'ko','MarkerSize',8,'LineWidth',2);
title('torque')

subplot(3,2,2); hold on;
plot(t,ddx)
plot(tGrid,ddxGrid,'ko','MarkerSize',8,'LineWidth',2);
title('acceleration')

subplot(3,2,4); hold on;
plot(t,dddx)
plot(tGrid,dddxGrid,'ko','MarkerSize',8,'LineWidth',2);
title('jerk')

subplot(3,2,6); hold on;
plot(t,ddddx)
plot(tGrid,ddddxGrid,'ko','MarkerSize',8,'LineWidth',2);
title('snap')




