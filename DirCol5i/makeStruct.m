function StructOutput = makeStruct(varargin)
% StructOutput = makeStruct(varargin)
%
% A struct is created with the property that each field corresponds to one
% of the arguments passed to this function.
%
% Example:
%
% R = makeStruct(a,b,c,d,e);
%
% R.a == a;
% R.b == b;
% R.c == c;
% R.d == d;
% R.e == e;
%
% Notes:
% 
%   An arbitrary number of inputs are allowed, but their names should be
%   unique.
%

N_Inputs = length(varargin);

for i=1:N_Inputs
    name = inputname(i);
    StructOutput.(name) = varargin{i};
end

end