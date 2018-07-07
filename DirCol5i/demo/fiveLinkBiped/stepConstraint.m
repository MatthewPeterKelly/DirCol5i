function [c, ceq] = stepConstraint(boundary,p)
% [c, ceq] = stepConstraint(boundary,p)
%
% This function applies the non-linear boundary constraint to ensure that
% there gait is periodic, with the correct heel-strike map and left-right
% mapping.
%
% TODO:  finish documentation
%

% Gait must be periodic
ceq1 = cst_heelStrike([boundary.x0; boundary.dx0], [boundary.x1; boundary.dx1],p);

% Foot collision (toe-off and heel-strike) velocity
c = cst_footVel([boundary.x0; boundary.dx0], [boundary.x1; boundary.dx1],p);

% Step length and height
ceq2 = cst_stepLength(boundary.x1,p);

% Pack up equality constraints:
ceq = [ceq1;ceq2];




end
