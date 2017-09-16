function [t,w] = computeGaussPoints()
% [t,w] = computeGaussPoints()
%
% Computes the 4 gauss-lobatto or gauss-legendre points on the interval [-1,1]
%

global USE_LOBOTTO_POINTS

if USE_LOBOTTO_POINTS
    
    % Gauss-Lobotto
    t = [-1, -0.2*sqrt(5), 0.2*sqrt(5), 1];
    
    if nargout > 1
        w = [1/6, 5/6, 5/6, 1/6];
    end
    
else
    % Gauss-Legendre -- 4 points
    
    a = (2/7)*sqrt(6/5);
    b = 3/7;
    c = sqrt(b + a);
    d = sqrt(b - a);
    
    t = [-c,-d, d, c];
    
    if nargout > 1
        e = sqrt(30)/36;
        w = 0.5 + e*[-1,1,1,-1];
    end
        
end

end