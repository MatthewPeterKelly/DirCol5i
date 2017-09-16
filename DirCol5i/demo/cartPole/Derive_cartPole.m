% Derive_cartPole.m
%
% This script derives the equations of motion for a simple cart-pole
% system: A cart that travels on horizontal rails, with a pendulum hanging
% from it. A force actuator is used to move the car along the rails. Both
% the cart and the pendulum are point masses.
%

clear; clc;

syms x q dx dq ddx ddq dddx dddq ddddx ddddq 'real'   % states
syms u 'real' % actuation
syms m1 m2 g l 'real' % physical parameters

%%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e = cos(q)*(-j) + sin(q)*(i);    % cart -> tip of pendulum

%%%% State vectors:
z = [x;q;dx;dq];
dz = [dx;dq;ddx;ddq];

zKin = [x;q;dx;dq;ddx;ddq;dddx;dddq];
dzKin = [dx;dq;ddx;ddq;dddx;dddq;ddddx;ddddq];

%%%% Kinematics:
derivative = @(f)( jacobian(f,z)*dz );   %Chain rule!
derivKin = @(f)( jacobian(f,zKin)*dzKin );   %Chain rule!

p1 = x*i;  %Position of the center of the cart
p2 = p1 + l*e;  %Position of the end of the pendulum

dp1 = derivative(p1);  
dp2 = derivative(p2); 

ddp1 = derivative(dp1); 
ddp2 = derivative(dp2); 
normAccel1 = simplify(ddp1(1)^2 + ddp1(2)^2);
normAccel2 = simplify(ddp2(1)^2 + ddp2(2)^2);

dddp1 = derivKin(ddp1); 
dddp2 = derivKin(ddp2); 
normJerk1 = simplify(dddp1(1)^2 + dddp1(2)^2); 
normJerk2 = simplify(dddp2(1)^2 + dddp2(2)^2);

ddddp1 = derivKin(dddp1); 
ddddp2 = derivKin(dddp2); 
normSnap1 = simplify(ddddp1(1)^2 + ddddp1(2)^2);
normSnap2 = simplify(ddddp2(1)^2 + ddddp2(2)^2);



%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Horizontal force balance for entire system (+i direction)
sumForces1 = u;
sumInertial1 = m1*dot(ddp1,i) + m2*dot(ddp2,i);
eqn1 = sumForces1 - sumInertial1;

%%%% Angular momentum balance of pendulum about support point:
sumTorques2 = cross2d(p2-p1,-m2*g*j);   %Gravity torque on pendulum
sumInertial2 = cross2d(p2-p1,m2*ddp2);
eqn2 = sumTorques2 - sumInertial2;

%%%% Collect equations of motion:
eqns = simplify([eqn1;eqn2]);

%%%% Generate an optimized matlab function for dynamics:
matlabFunction(eqns,...
    'file','autoGen_cartPoleDynamics.m',...
    'vars',{ddx,q,dq,ddq,u,m1,m2,g,l},...
    'outputs',{'eqns'});

%%%% Generate a function for computing the kinematics:
syms empty 'real'  %fixes a bug in matlabFunction related to vectorization
p1(2) = p1(2) + empty;
dp1(2) = dp1(2) + empty;
matlabFunction(p1,p2,dp1,dp2,...
    'file','autoGen_cartPoleKinematics.m',...
    'vars',{x,q,dx,dq,l,empty},...
    'outputs',{'p1','p2','dp1','dp2'});

%%%% Compute the kinematic objective functions:
matlabFunction(normAccel1,normAccel2,...
    'file','autoGen_normBobAccel.m',...
    'vars',{q, dq, ddx, ddq,  l},...
    'outputs',{'normAccel1','normAccel2'});
matlabFunction(normJerk1, normJerk2,...
    'file','autoGen_normBobJerk.m',...
    'vars',{q, dq,ddq, dddx, dddq, l},...
    'outputs',{'normJerk1','normJerk2'});
matlabFunction(normSnap1, normSnap2,...
    'file','autoGen_normBobSnap.m',...
    'vars',{q,  dq,  ddq,  dddq, ddddx, ddddq, l},...
    'outputs',{'normSnap1', 'normSnap2'});

