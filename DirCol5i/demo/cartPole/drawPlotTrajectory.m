function drawPlotTrajectory(output)
% drawPlotTrajectory(output)
%
% Draws a stop-action swing-up for the trajectory

soln = output.soln(end);
p = output.problem.auxdata.dynParam;

tGrid = soln.colPts.t;

t = linspace(tGrid(1), tGrid(end), 150);
X = ppval(soln.pp.x,t);
dX = ppval(soln.pp.dx,t);
[p1,p2,dp1,dp2] = cartPoleKinematics([X; dX],p);
s1 = sqrt(dp1(1,:).^2 + dp1(2,:).^2);
s2 = sqrt(dp2(1,:).^2 + dp2(2,:).^2);

% Draw swing-up
nFrame = 9;  %Number of frames to draw
subplot(1,2,1)
drawCartPoleTraj(t,p1,p2,nFrame,'small');

% x vs t
subplot(1,2,2); hold on;
plot(t,s1,'k--')
plot(t,s2)
xlabel('time')
ylabel('speed')
legend('cart','bob','Location','SouthEast')

end