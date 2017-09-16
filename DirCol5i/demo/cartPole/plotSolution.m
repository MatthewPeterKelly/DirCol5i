function plotSolution(output)
soln = output.soln(end);
func = output.problem.func;

tGrid = soln.colPts.t;
uGrid = soln.colPts.u;
duGrid = soln.colPts.du;

xGrid = soln.colPts.x(1,:);
dxGrid = soln.colPts.dx(1,:);
ddxGrid = soln.colPts.ddx(1,:);

qGrid = soln.colPts.x(2,:);
dqGrid = soln.colPts.dx(2,:);
ddqGrid = soln.colPts.ddx(2,:);

t = linspace(tGrid(1), tGrid(end), 150);
X = ppval(soln.pp.x,t);
dX = ppval(soln.pp.dx,t);
ddX = ppval(soln.pp.ddx,t);
dddX = ppval(soln.pp.dddx,t);
ddddX = ppval(soln.pp.ddddx,t);
u = ppval(soln.pp.u,t);
du = ppval(soln.pp.du,t);
x = X(1,:);
q = X(2,:);
dx = dX(1,:);
dq = dX(2,:);
ddx = ddX(1,:);
ddq = ddX(2,:);

% Compute the cost function along the trajectory:
cost = func.pathObj(t,X,dX,ddX,u,  dddX, ddddX, du);

%%%% Plot the solution at the grid points:

subplot(3,3,1); hold on;
plot(t, x, 'b-','LineWidth', 2);
plot(tGrid, xGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('x')
title('cart position')

subplot(3,3,4); hold on;
plot(t, dx, 'b-','LineWidth', 2);
plot(tGrid, dxGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('dx')
title('cart velocity')

subplot(3,3,7); hold on;
plot(t, ddx, 'b-','LineWidth', 2);
plot(tGrid, ddxGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('ddx')
title('cart acceleration')


subplot(3,3,2); hold on;
plot(t, q, 'b-','LineWidth', 2);
plot(tGrid, qGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('q')
title('pole angle')

subplot(3,3,5); hold on;
plot(t, dq, 'b-','LineWidth', 2);
plot(tGrid, dqGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('dq')
title('pole rate')

subplot(3,3,8); hold on;
plot(t, ddq, 'b-','LineWidth', 2);
plot(tGrid, ddqGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('ddq')
title('pole acceleration')



subplot(3,3,3); hold on;
plot(t, u, 'b-','LineWidth', 2);
plot(tGrid, uGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('u')
title('applied force')

subplot(3,3,6); hold on;
plot(t, du, 'b-','LineWidth', 2);
plot(tGrid, duGrid, 'ko', 'MarkerSize',6, 'LineWidth', 2);
xlabel('t')
ylabel('du')
title('force rate')

subplot(3,3,9); hold on;
plot(t, cost, 'b-','LineWidth', 2);
xlabel('t')
ylabel('dJ')
title('cost integrand')



end