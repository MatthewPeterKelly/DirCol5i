% TEST - get collocation points for control
%
%

clc; clear;

%%%% Analytic test function
uFun = @(t)( [sin(t); exp(-t).*cos(t)] );
duFun = @(t)( [cos(t); -exp(-t).*cos(t) - exp(-t).*sin(t)] );

%%%% Compute position and velocity at knot points
tKnot = 4*[0.0,  0.4,  0.7,  0.9,  1.0];
tLow = tKnot(1);
tUpp = tKnot(end);
uKnot = uFun(tKnot);
duKnot = duFun(tKnot);

%%%% Evaluate at the collocation points:
[tCol, uCol, duCol] = getColPtControl(tKnot,uKnot,duKnot);

%%%% Compute the spline fits:
 [PPu, PPdu] = getPpControl(tKnot,uKnot,duKnot);

%%%% Plots!
figure(486); clf;
tFun = linspace(tLow, tUpp, 100);

subplot(2,1,1); hold on;
plot(tFun, uFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPu,tFun), 'm.','MarkerSize',6);
plot(tKnot, uKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, uCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('u')

subplot(2,1,2); hold on;
plot(tFun, duFun(tFun), 'k-', 'LineWidth',2);
plot(tFun, ppval(PPdu,tFun), 'm.','MarkerSize',6);
plot(tKnot, duKnot, 'bo', 'MarkerSize',10,'LineWidth',2);
plot(tCol, duCol, 'rs', 'MarkerSize',8,'LineWidth',2);
ylabel('du')
