% Derive_acrobot.m
%
% This script derives the equations of motion for the simple acrobot robot:
% a double pendulum with a motor at the elbow joint. 
%

syms q1 q2 dq1 dq2 ddq1 ddq2 dddq1 dddq2 ddddq1 ddddq2 'real'   % states
syms u 'real' % actuation
syms m1 m2 g l1 l2 'real' % physical parameters

%%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e1 = cos(q1)*(-j) + sin(q1)*(i);    % shoulder -> elbow
e2 = cos(q2)*(-j) + sin(q2)*(i);    % elbow -> wrist

%%%% State vectors:
z =  [q1; q2; dq1; dq2; ddq1; ddq2; dddq1; dddq2];
dz = [dq1;dq2;ddq1;ddq2;dddq1;dddq2;ddddq1;ddddq2];
derivative = @(x)( simplify(jacobian(x,z)*dz) );    %Chain rule

%%%% Kinematics:
p1 = l1*e1;
p2 = p1 + l2*e2;

dp1 = derivative(p1);  %Derivative to get velocity of elbow joint
dp2 = derivative(p2); 

ddp1 = derivative(dp1);  %Derivative to get accelerations
ddp2 = derivative(dp2); 

dddp1 = derivative(ddp1);  %Derivative to get jerk (derivative of accel)
dddp2 = derivative(ddp2); 

ddddp1 = derivative(dddp1);  %Derivative to get snap (derivative of jerk)
ddddp2 = derivative(dddp2); 


%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Angular momentum balance of system about shoulder joint:
sumTorques1 = cross2d(p1,-m1*g*j) + cross2d(p2,-m2*g*j);
sumInertial1 = cross2d(p1,m1*ddp1) + cross2d(p2,m2*ddp2);
eqn1 = sumTorques1-sumInertial1;

%%%% Angular momentum balance of outer link about elbow joint:
sumTorques2 = cross2d(p2-p1,-m2*g*j) + u;
sumInertial2 = cross2d(p2-p1,m2*ddp2);
eqn2 = sumTorques2-sumInertial2;

%%%% Matlab function for implicit second-order dynamics
matlabFunction(eqn1,eqn2,...
    'file','autoGen_implicitDynamics.m',...
    'vars',{q1,q2,dq1,dq2,ddq1,ddq2,u,m1,m2,g,l1,l2},...
    'outputs',{'eqn1','eqn2'});

%%%% Compute the energy of the system:
U = m1*g*dot(p1,j) + m2*g*dot(p2,j);   %Potential Energy
T = 0.5*m1*dot(dp1,dp1) + 0.5*m2*dot(dp2,dp2);   %Kinetic Energy

%%%% Generate an optimized matlab function for energy:
matlabFunction(U,T,...
    'file','autoGen_acrobotEnergy.m',...
    'vars',{q1,q2,dq1,dq2,m1,m2,g,l1,l2},...
    'outputs',{'U','T'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p2,dp1,dp2,...
    'file','autoGen_acrobotKinematics.m',...
    'vars',{q1,q2,dq1,dq2,l1,l2},...
    'outputs',{'p1','p2','dp1','dp2'});

matlabFunction(ddp1,ddp2,...
    'file','autoGen_acrobotPosAccel.m',...
    'vars',{q1, q2, dq1, dq2, ddq1, ddq2,l1,l2},...
    'outputs',{'ddp1','ddp2'});

matlabFunction(dddp1,dddp2,...
    'file','autoGen_acrobotPosJerk.m',...
    'vars',{q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2,l1,l2},...
    'outputs',{'dddp1','dddp2'});

matlabFunction(ddddp1,ddddp2,...
    'file','autoGen_acrobotPosSnap.m',...
    'vars',{q1, q2, dq1, dq2, ddq1, ddq2, dddq1, dddq2, ddddq1, ddddq2, l1,l2},...
    'outputs',{'ddddp1','ddddp2'});








