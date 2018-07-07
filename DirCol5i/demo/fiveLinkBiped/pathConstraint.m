function [c, ceq] = pathConstraint(x)
% [c, ceq] = pathConstraint(x)
%
% This function implements a simple path constraint to keep the knee joint
% of the robot from hyer-extending.
%
% TODO:  finish documentation!
%

q1 = x(1,:);
q2 = x(2,:);
q4 = x(4,:);
q5 = x(5,:);

c = [...
    q1-q2;    % Stance knee joint limit
    q5-q4];   % Swing knee joint limit

ceq = [];


end
