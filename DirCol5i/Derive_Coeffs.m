function Derive_Coeffs()
%
% Derive the equations that are used to compute the collocation points
%
% The state and control at the upper boundary is given by xB and uB, while
% the state and control ad the lower boundary is given by xA and uA;
%

deriveCollocationState();
deriveCollocationControl();

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [C,t,x,dx,ddx,dddx,ddddx] = getPolynomial(n)
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
dddx = diff(ddx,t);
ddddx = diff(dddx,t);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function deriveCollocationState()

order = 5;
[C,t,x,dx,ddx,dddx,ddddx] = getPolynomial(order);

syms xA dxA ddxA xB dxB ddxB 'real'
tA = sym(-1);
tB = sym(1);

eqn1 = subs(x,t,tA) - xA;
eqn2 = subs(dx,t,tA) - dxA;
eqn3 = subs(ddx,t,tA) - ddxA;
eqn4 = subs(x,t,tB) - xB;
eqn5 = subs(dx,t,tB) - dxB;
eqn6 = subs(ddx,t,tB) - ddxB;
eqns = [eqn1;eqn2;eqn3;eqn4;eqn5;eqn6];
[A,b] = equationsToMatrix(eqns,C);
soln = A\b;

X = subs(x,{'C1','C2','C3','C4','C5','C6'},{soln(1),soln(2),soln(3),soln(4),soln(5),soln(6)});
dX = subs(dx,{'C1','C2','C3','C4','C5','C6'},{soln(1),soln(2),soln(3),soln(4),soln(5),soln(6)});
ddX = subs(ddx,{'C1','C2','C3','C4','C5','C6'},{soln(1),soln(2),soln(3),soln(4),soln(5),soln(6)});
dddX = subs(dddx,{'C1','C2','C3','C4','C5','C6'},{soln(1),soln(2),soln(3),soln(4),soln(5),soln(6)});
ddddX = subs(ddddx,{'C1','C2','C3','C4','C5','C6'},{soln(1),soln(2),soln(3),soln(4),soln(5),soln(6)});

%% %% Write out functions

matlabFunction(...
    X,dX,ddX,dddX,ddddX,...
    'file','autoGen_getColPtState.m',...
    'vars',{t,xA,xB,dxA,dxB,ddxA,ddxB},...
    'outputs',{...    
    'X','dX','ddX','dddX','ddddX'});

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function deriveCollocationControl()

order = 3;
[C,t,u,du] = getPolynomial(order);

syms uA duA dduA uB duB dduB 'real'
tA = sym(-1);
tB = sym(1);

eqn1 = subs(u,t,tA) - uA;
eqn2 = subs(du,t,tA) - duA;
eqn3 = subs(u,t,tB) - uB;
eqn4 = subs(du,t,tB) - duB;
eqns = [eqn1;eqn2;eqn3;eqn4];
[A,b] = equationsToMatrix(eqns,C);
soln = A\b;

U = subs(u,{'C1','C2','C3','C4'},{soln(1),soln(2),soln(3),soln(4)});
dU = subs(du,{'C1','C2','C3','C4'},{soln(1),soln(2),soln(3),soln(4)});


%% %% Write out functions

matlabFunction(...
    U, dU,...
    'file','autoGen_getColPtControl.m',...
    'vars',{t,uA,uB,duA,duB},...
    'outputs',{...    
    'U', 'dU'});

end