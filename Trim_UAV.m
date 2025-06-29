clc
close all

% Trim Solver Script
% This script finds the trim conditions (states and control inputs)
% that minimize the cost function defined in cost_function.m
% using the Nelder-Mead optimization (fminsearch).

% Initial guess for Z vector
% Z = [states(1:9); inputs(10:13)]
Z_guess = zeros(13, 1);  
Z_guess(1)  = 11.2;    % Initial forward position (pn) or body velocity (u)
Z_guess(13) = 1;     % Initial guess for throttle

% Initialize convergence flag
f0 = 1;  % Initial cost value (arbitrary non-zero to enter loop)
% Optimization loop using fminsearch
while (f0 > 1e-10)
    
    % Minimize the cost function using current guess
    [ZStar, f0] = fminsearch('cost_function', Z_guess, ...
        optimset('Tolx', 1e-10, 'MaxFunEvals', 10000, 'MaxIter', 10000));
    % Update guess with optimized values for next iteration
    Z_guess = ZStar;

end
% Extract trimmed state and input vectors
x_trim = ZStar(1:9);      % Trimmed states
u_trim = ZStar(10:13);    % Trimmed control inputs
save('trim_results.mat', 'x_trim', 'u_trim', 'ZStar');

% Optional: Display results
disp('Trimmed States (x_trim):');
disp(x_trim);
disp('Trimmed Inputs (u_trim):');
disp(u_trim);
disp('Final Cost Value:');
disp(f0);
