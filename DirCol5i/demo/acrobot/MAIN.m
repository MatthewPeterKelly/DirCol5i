%MAIN.m  --  solve swing-up problem for acrobot
%
% This script finds the minimum torque-squared trajectory to swing up the
% acrobot robot: a double pendulum with a motor between the links
%
%

clc; clear;
addpath ../../

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
dyn.m1 = 1;  % elbow mass
dyn.m2 = 1; % wrist mass
dyn.g = 9.81;  % gravity
dyn.l1 = 0.5;   % length of first link
dyn.l2 = 0.5;   % length of second link

maxTorque = inf;  % Max torque at the elbow

objectiveFunction = 'snap';   %Must match a field name in objFun

problem.auxdata.dyn = dyn;
problem.auxdata.maxTorque = maxTorque;
problem.auxdata.objFunName = objectiveFunction;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Objective Functions                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


objFun.torque = @(t,x,dx,ddx,u,dddx,ddddx,du)( u.^2 ); 
objFun.torqueRate = @(t,x,dx,ddx,u,dddx,ddddx,du)( du.^2 ); 
objFun.accel = @(t,x,dx,ddx,u,dddx,ddddx,du)(  objFunAccel(x,dx,ddx,dyn)  );
objFun.jerk = @(t,x,dx,ddx,u,dddx,ddddx,du)(  objFunJerk(x,dx,ddx,dddx,dyn)  );
objFun.snap = @(t,x,dx,ddx,u,dddx,ddddx,du)(  objFunSnap(x,dx,ddx,dddx,ddddx,dyn)  );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,dx,ddx,u)( acrobotDynamics(x,dx,ddx,u,dyn) );

problem.func.pathObj = objFun.(objectiveFunction);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t0 = 0;  
tF = 2;  %For now, force it to take exactly this much time.

x0 = [0;0];   %Initial state: hanging down
xF = pi*[1;1]; %Final state: inverted balance

dx0 = [0;0]; 
dxF = [0;0]; 

problem.bounds.t0.low = t0;
problem.bounds.t0.upp = t0;
problem.bounds.t1.low = tF;
problem.bounds.t1.upp = tF;

problem.bounds.x0.low = x0;  %Stable equilibrium
problem.bounds.x0.upp = x0;
problem.bounds.x1.low = xF; %Inverted balance
problem.bounds.x1.upp = xF;

problem.bounds.dx0.low = dx0;  %Stable equilibrium
problem.bounds.dx0.upp = dx0;
problem.bounds.dx1.low = dxF; %Inverted balance
problem.bounds.dx1.upp = dxF;

problem.bounds.u.low = -maxTorque;
problem.bounds.u.upp = maxTorque;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values

problem.guess.t = [t0, tF];
problem.guess.x = [x0, xF];
problem.guess.dx = [dx0, dxF];
problem.guess.u = [0, 0];


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Options:                                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.nlpOpt = optimset(...
    'MaxIter',2500,...
    'Display','iter',...
    'MaxFunEvals',1e5);

problem.options.mesh.nSegmentInit = 5;
problem.options.mesh.maxIter = 3;
problem.options.mesh.maxSubDivide = 3;
problem.options.mesh.tol = 1e-3;


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Solve!                                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

output = dirCol5i(problem);


%% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

tSpan = output.soln(end).knotPts.t([1,end]);
t = linspace(tSpan(1), tSpan(end), 150);
z = ppval(output.soln(end).pp.x, t);
dz = ppval(output.soln(end).pp.dx, t);
ddz = ppval(output.soln(end).pp.dx, t);
dddz = ppval(output.soln(end).pp.dx, t);
ddddz = ppval(output.soln(end).pp.dx, t);
u = ppval(output.soln(end).pp.u, t);
du = ppval(output.soln(end).pp.u, t);

%HINT:  type help animate to figure out how to use the keyboard to interact
%with the animation (slow motion, pause, jump forward / backward...)

%% Animate the results:
A.plotFunc = @(t,z)( drawAcrobot(t,z,dz,dyn) );
A.speed = 0.25;
A.figNum = 101;
animate(t,z,A)

%% Plot the state
figure(1337); clf; plotAcrobot(t,z,dz,u,dyn);

%% Plot the kinematics:
figure(1523); clf;
vecNorm = @(p)( sqrt(p(1,:).^2 + p(2,:).^2) );
[p1,p2,dp1,dp2] = acrobotKinematics(z,dz,dyn);
ds1 = vecNorm(dp1);  ds2 = vecNorm(dp2);
[ddp1,ddp2] = acrobotPosAccel(z,dz,ddz,dyn);
dds1 = vecNorm(ddp1);  dds2 = vecNorm(ddp2);
[dddp1,dddp2] = acrobotPosJerk(z,dz,ddz,dddz,dyn);
ddds1 = vecNorm(dddp1);  ddds2 = vecNorm(dddp2);
[ddddp1,ddddp2] = acrobotPosSnap(z,dz,ddz,dddz,ddddz,dyn);
dddds1 = vecNorm(ddddp1);  dddds2 = vecNorm(ddddp2);
c1 = [0.8,0.2,0.2];
c2 = [0.2,0.2,0.8];
c3 = [0.2,0.8,0.2];

subplot(2,3,1); hold on;
plot(0,0,'ko','MarkerSize',12,'LineWidth',3);
plot(p1(1,:),p1(2,:),'Color',c1);
plot(p2(1,:),p2(2,:),'Color',c2);
xlabel('x')
ylabel('y')
title('traces')

subplot(2,3,4); hold on;
plot(t,du,'Color',c3)
xlabel('t')
ylabel('du')
title('torque rate')

subplot(2,3,2); hold on;
plot(t,ds1,'Color',c1);
plot(t,ds2,'Color',c2);
xlabel('t')
ylabel('ds')
title('speed')

subplot(2,3,3); hold on;
plot(t,ddds1,'Color',c1);
plot(t,ddds2,'Color',c2);
xlabel('t')
ylabel('ddds')
title('jerk')


subplot(2,3,5); hold on;
plot(t,dds1,'Color',c1);
plot(t,dds2,'Color',c2);
xlabel('t')
ylabel('dds')
title('accel')

subplot(2,3,6); hold on;
plot(t,dddds1,'Color',c1);
plot(t,dddds2,'Color',c2);
xlabel('t')
ylabel('dddds')
title('snap')






