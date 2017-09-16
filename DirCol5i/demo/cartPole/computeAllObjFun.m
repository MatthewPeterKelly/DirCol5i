function allObjFun = computeAllObjFun(output, objFun, display)
% allObjFun = computeAllObjFun(output, objFun, display)
%
% computes the value of all possible objective functions

names = fieldnames(objFun);

t = output.soln(end).colPts.t;
w = output.soln(end).colPts.w;
x = output.soln(end).colPts.x;
dx = output.soln(end).colPts.dx;
ddx = output.soln(end).colPts.ddx;
dddx = output.soln(end).colPts.dddx;
ddddx = output.soln(end).colPts.ddddx;
u = output.soln(end).colPts.u;
du = output.soln(end).colPts.du;

disp('======================================')
disp('All objective function values: ')
for i=1:length(names)
    f = objFun.(names{i})(t,x,dx,ddx,u,dddx,ddddx,du);
    quadSum = sum(f.*w);
    allObjFun.(names{i}) = quadSum;
    if display
        fprintf('%s:  %4.4e \n',names{i},quadSum);
    end
end

end