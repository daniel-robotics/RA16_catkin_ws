%% Constructs a polynomial (cubic) curve between two points
%  Equation is optimized for embedded devices:
%   - Total of 6 multiplications and 6 additions for function eval
%   - Suitable for fixed-point math on 32-bit architecture

%% Fixed Point Settings
T = numerictype(1,32,16);
F = fimath('RoundingMethod', 'Floor',       ...
           'OverflowAction', 'Wrap',        ...
           'SumMode', 'SpecifyPrecision',   ...
           'SumWordLength', 32,             ...
           'SumFractionLength', 16,         ...
           'ProductMode', 'SpecifyPrecision',   ...
           'ProductWordLength', 32,         ...
           'ProductFractionLength', 16);
P = fipref('NumberDisplay', 'RealWorldValue', ...
           'NumericTypeDisplay', 'none',    ...
           'FimathDisplay', 'none');

%% Initial and final conditions
th_i  = fi(0,  T,F);	% Initial (t=0) value (rad)
thd_i = fi(0.1,   T,F);	% Initial (t=0) slope (rad/s)
th_f  = fi(1, T,F);	% Final (t=tf) value  (rad)
thd_f = fi(0.5,   T,F);	% Final (t=tf) slope  (rad/s)
tf    = fi(0.1, T,F); % Time offset for final conditions (s)

%% Range and precision
sample_rate = 50;           % Rate of motor control loop (Hz)
num_samples = 5;            % Number of samples to look back for velocity estimation
pre_t   = 1/sample_rate;    % Time precision (s)
lim_tf  = 1.0;              % Max time allowable for tf (s)

lim_th  = 2*pi;             % Max value for th (rad)
pre_th  = deg2rad(1/11.667);% Smallest measurable increment of th (rad)

lim_thd = 2*pi;             % Max value for thd (rad/s)
pre_thd = pre_th/(pre_t*num_samples); % Smallest measurable increment of thd (rad/s)



%% Precomputed constants
%  Only need to calculate once for a particular curve
a1 = -2*(th_f-th_i);
a2 = 3*(th_f-th_i);
a3 = thd_f + thd_i;
a4 = 1/tf;
a5 = th_i;
a6 = thd_i;

%% Evaluate curve over domain t=[0,tf]
t_list = [];
x_list = [];

t=fi(0,T,F);
while t<=tf
    x = fcn(t, a1, a2, a3, a4, a5, a6);
    t_list(end+1) = double(t);
    x_list(end+1) = double(x);
    t = t+pre_t;
end

plot(t_list, x_list);

%% Test function over entire allowable range of parameters (slow)
% for th_i = -lim_th : 1 : lim_th+1
%     for th_f = th_i+pre_th : 1 : lim_th+1
%         for thd_i = -lim_thd : 1 : lim_thd+1
%             for thd_f = thd_i+pre_thd : 1 : lim_thd+1
%                 tf = 1;
%                 
%                 fprintf('Testing: th_i=%.4f, th_f=%.4f, thd_i=%.4f, thd_f=%.4f, tf=%.4f\n',th_i,th_f,thd_i,thd_f,tf);
%                 
%                 a1 = -2*(fi(th_f,T,F)-fi(th_i,T,F));
%                 a2 = 3*(fi(th_f,T,F)-fi(th_i,T,F));
%                 a3 = fi(thd_f,T,F) + fi(thd_i,T,F);
%                 a4 = 1/fi(tf,T,F);
%                 a5 = fi(th_i,T,F);
%                 a6 = fi(thd_i,T,F);
%                 t=fi(0,T,F);
%                 while t<=tf
%                     th = theta(t, a1, a2, a3, a4, a5, a6);
%                     t = t+pre_t;
%                 end
%                 
%             end
%         end
%     end
% end

%% Compute curve at point t
function x = fcn(t, a1, a2, a3, a4, a5, a6)
    r = a4*t;	% better than t/tf
    b1 = a3*t; 
    b2 = a6*t;
    x = r*(r*(a1*r + b1 + a2) - b1 - b2) + b2 + a5;
    
    % Check intermediate calculations for overflows
    S16_MAX = 32767;            % Max integer representable by Q16.16 format
    assert(abs(a4.data*t.data) < S16_MAX);
    assert(abs(a3.data*t.data) < S16_MAX);
    assert(abs(a6.data*t.data) < S16_MAX);
    assert(abs(a1.data*r.data) < S16_MAX);
    assert(abs(a1.data*r.data+b1.data+b2.data) < S16_MAX);
    assert(abs(r.data*(a1.data*r.data+b1.data+b2.data)) < S16_MAX);
    assert(abs(r.data*(a1.data*r.data+b1.data+b2.data)-b1.data-b2.data) < S16_MAX);
    assert(abs(r.data*(r.data*(a1.data*r.data+b1.data+b2.data)-b1.data-b2.data)) < S16_MAX);
    assert(abs(r.data*(r.data*(a1.data*r.data+b1.data+b2.data)-b1.data-b2.data)+b2.data+a5.data) < S16_MAX);
end