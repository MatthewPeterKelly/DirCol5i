% TEST - get collocation points for state
%
%

clc; clear;

%%%% Analytic test function
xFun = @(t)( [sin(t); exp(-t).*cos(t)] );
dxFun = @(t)( [cos(t); -exp(-t).*cos(t) - exp(-t).*sin(t)] );
ddxFun = @(t)( [-sin(t); 2.*exp(-t).*sin(t)] );
dddxFun = @(t)( [-cos(t); 2.*exp(-t).*cos(t) - 2.*exp(-t).*sin(t)] );
ddddxFun = @(t)( [sin(t); -4.*exp(-t).*cos(t)] );

%%%% Compute position, velocity, and acceleration at knot points
tKnot = 3*[0.0,  0.4,  0.7,  0.9,  1.0];
tLow = tKnot(1);
tUpp = tKnot(end);
xKnot = xFun(tKnot);
dxKnot = dxFun(tKnot);
ddxKnot = ddxFun(tKnot);
dddxKnot = dddxFun(tKnot);
ddddxKnot = ddddxFun(tKnot);

%%%% Evaluate at the collocation points:
[tCol, wCol, xCol, dxCol, ddxCol, dddxCol, ddddxCol] = getColPtState(tKnot,xKnot,dxKnot,ddxKnot);

%%%% Check integral calculations:
quadSoln = integral(xFun,tLow,tUpp,'ArrayValued',true);
nx = 2; W = ones(nx,1)*wCol;
quadCol = sum(W.*xCol,2);
quadErr = quadSoln - quadCol %#ok<NOPTS>

%%%% Compute the spline approximations:
[PPx, PPdx, PPddx, PPdddx, PPddddx] = getPpState(tKnot,xKnot,dxKnot,ddxKnot);

%%%% Plots!
figure(958); clf;
tFun = linspace(tLow, tUpp, 100);

subplot(3,2,1); hold on;
plot(tFun, xFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPx,tFun), 'm.','MarkerSize',6);
plot(tKnot, xKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, xCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('x')

subplot(3,2,3); hold on;
plot(tFun, dxFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPdx,tFun), 'm.','MarkerSize',6);
plot(tKnot, dxKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, dxCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('dx')

subplot(3,2,5); hold on;
plot(tFun, ddxFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPddx,tFun), 'm.','MarkerSize',6);
plot(tKnot, ddxKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, ddxCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('ddx')


subplot(3,2,2); hold on;
plot(tFun, dddxFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPdddx,tFun), 'm.','MarkerSize',6);
plot(tKnot, dddxKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, dddxCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('dddx')

subplot(3,2,4); hold on;
plot(tFun, ddddxFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPddddx,tFun), 'm.','MarkerSize',6);
plot(tKnot, ddddxKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, ddddxCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('ddddx')
