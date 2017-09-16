function drawTrajectory(output)
% drawTrajectory(output)
%
% Draws a stop-action swing-up for the trajectory

soln = output.soln(end);
p = output.problem.auxdata.dynParam;

tGrid = soln.colPts.t;

t = linspace(tGrid(1), tGrid(end), 150);
X = ppval(soln.pp.x,t);
dX = ppval(soln.pp.dx,t);
[p1,p2] = cartPoleKinematics([X; dX],p);

nFrame = 9;  %Number of frames to draw
drawCartPoleTraj(t,p1,p2,nFrame);


end