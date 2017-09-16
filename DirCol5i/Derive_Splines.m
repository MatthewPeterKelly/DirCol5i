function Derive_Splines()
%
% Derive the equations that are usedto construct the splines to interpolate
% the solution.
%
% Here we assume that the function is defined over the domain [tA, tB]
%
% The state and control at the upper boundary is given by xB and uB, while
% the state and control ad the lower boundary is given by xA and uA;
%

deriveStateSpline();
deriveControlSpline();

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [C,t,x,dx,ddx] = getPolynomial(n)
%
% y(t) = C1 + C2*t + C3*t^2 + ...
%
% INPUTS:
%   n = Order of the polynomial
%
% OUTPUTS:  (all symbolic)
%   C = [n+1, 1] = vector of coefficients

C = sym('C',[n+1,1],'real');  %Coefficients of polynomial
t = sym('t', 'real');  %continuous time   t0 <= t < t1

x = sym(0);
for i = 1:(n+1)
    x = x + C(i)*t^(i-1);
end
dx = diff(x,t);
ddx = diff(dx,t);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function deriveStateSpline()

order = 5;
[C,t,x,dx,ddx] = getPolynomial(order);

syms tA tB xA dxA ddxA xB dxB ddxB 'real'

eqn1 = subs(x,t,tA) - xA;
eqn2 = subs(dx,t,tA) - dxA;
eqn3 = subs(ddx,t,tA) - ddxA;
eqn4 = subs(x,t,tB) - xB;
eqn5 = subs(dx,t,tB) - dxB;
eqn6 = subs(ddx,t,tB) - ddxB;
eqns = [eqn1;eqn2;eqn3;eqn4;eqn5;eqn6];
[A,b] = equationsToMatrix(eqns,C);
soln = A\b;

matlabFunction(soln(1),soln(2),soln(3),soln(4),soln(5),soln(6),...
    'file','autoGen_stateSplineCoeffs.m',...
    'vars',{tA,tB,xA,xB,dxA,dxB,ddxA,ddxB},...
    'outputs',{'C1','C2','C3','C4','C5','C6'});


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function deriveControlSpline()

order = 3;
[C,t,u,du] = getPolynomial(order);

syms tA tB uA duA uB duB  'real'

eqn1 = subs(u,t,tA) - uA;
eqn2 = subs(du,t,tA) - duA;
eqn3 = subs(u,t,tB) - uB;
eqn4 = subs(du,t,tB) - duB;
eqns = [eqn1;eqn2;eqn3;eqn4];
[A,b] = equationsToMatrix(eqns,C);
soln = A\b;

matlabFunction(soln(1),soln(2),soln(3),soln(4),...
    'file','autoGen_controlSplineCoeffs.m',...
    'vars',{tA,tB,uA,uB,duA,duB},...
    'outputs',{'C1','C2','C3','C4'});


end
