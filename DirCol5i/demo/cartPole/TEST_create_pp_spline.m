% TEST  --  create a pp spline from scratch
%
%
% 
% pp = 
% 
%       form: 'pp'
%     breaks: [0 0.2222 0.4444 0.6667 0.8889 1.1111 1.3333 1.5556 1.7778 2]
%      coefs: [18x4 double]
%     pieces: 9
%      order: 4
%        dim: 2
%        
  
%% Linear, Scalar

ppTest1.form = 'pp';
ppTest1.breaks = [0,1,2,3];   %L = 3 segments
ppTest1.coefs = [...    %K = order 2 (linear)
    1,0;   %Slope, Offset
    0,1;
    2,0];
ppTest1.pieces = 3;   %L
ppTest1.order = 2;  %K
ppTest1.dim = 1;  %Scalar data for now

time = linspace(ppTest1.breaks(1), ppTest1.breaks(end),  100);
value = ppval(ppTest1,time);

figure(1); clf;
plot(time,value);

       
%% Linear, Vector



ppTest2.form = 'pp';
ppTest2.breaks = [0,1,3,5];   %L = 3 segments
ppTest2.coefs = [...    %K = order 2 (linear)
    
    -1,0;   %First segment   first dimension
    0,-1;  %                 second dimension
    
    -2,0;    % second segment
    1,0;  
 
    0,1;   % third segment
    2,0];

ppTest2.pieces = 3;   %L
ppTest2.order = 2;  %K
ppTest2.dim = 2;  %Scalar data for now

time = linspace(ppTest2.breaks(1), ppTest2.breaks(end),  100);
value = ppval(ppTest2,time);

figure(2); clf;
plot(time,value);

       

%% quadratic, Vector



ppTest2.form = 'pp';
ppTest2.breaks = [0,1,3,5];   %L = 3 segments
ppTest2.coefs = [...    %K = order 2 (linear)
    
    1, 0,0;   %First segment   first dimension
    0, 0,-1;  %                 second dimension
    
    0, -2,0;    % second segment
    0, 1,0;  
 
    -2, 0,0;   % third segment
    0, 2,0];

ppTest2.pieces = 3;   %L
ppTest2.order = 3;  %K
ppTest2.dim = 2;  %Scalar data for now

time = linspace(ppTest2.breaks(1), ppTest2.breaks(end),  100);
value = ppval(ppTest2,time);

figure(3); clf;
plot(time,value);

       