% File: test_jacob.m
% Description: Tests the corrected Jacobian calculation function jacob.m

clc; clear;

% Define test configurations [theta, d]
test_cases = [
    0, 0;        % Initial configuration
    pi/2, 0;     % 90-degree rotation
    pi, 0;       % 180-degree rotation
    -pi/2, 0;    % -90-degree rotation
    pi/4, 1;     % 45-degree rotation with extension
];

% Display results
disp('Testing Corrected jacob(q)...');

% Loop through each test case
for i = 1:size(test_cases, 1)
    q = test_cases(i, :)'; % Extract joint configuration as a column vector
    J = jacob(q);          % Call Jacobian function
    
    % Display results
    fprintf('Test Case %d: theta=%.2f, d=%.2f\n', i, q(1), q(2));
    disp('Jacobian:');
    disp(J);
    fprintf('-----------------------------\n');
end
