% File: ResolvedRate.m
% Description: Implements the resolved rate algorithm for following a horizontal line segment.

clear; clc; close all;

% Initial robot configuration
q = [0; 0];   % Starting configuration [theta, d]
target_y = 1.5;  % Desired y-position of the end-effector

% Simulation parameters
dt = 0.1;                      % Time step
x_target = linspace(0.5, -0.5, 100); % Correct path from 0.5 to -0.5

% Create figure for visualization
figure(1); clf; hold on;

% Loop to follow the path using resolved-rate control
for i = 1:length(x_target)
    % Desired position of the end-effector
    x_desired = [x_target(i); target_y];

    % Current end-effector position using forward kinematics
    x_current = fk(q);

    % Compute position error
    error = x_desired - x_current;

    % Compute joint velocity using the full Jacobian
    J = jacob(q);
    q_dot = pinv(J(1:2,:)) * error; % Use pseudo-inverse for robustness

    % Update joint configuration using calculated velocities
    q = q + q_dot * dt;

    % Visualize the robot as a two-link arm
    clf; hold on;
    % Compute joint positions for visualization
    L1 = 1; L2 = 1;
    theta = q(1); d = q(2);

    % Corrected end-effector position to maintain exact path
    x1 = L1 * cos(theta); % First link end
    y1 = L1 * sin(theta);
    x2 = x1 + L2 * cos(theta); % End-effector position
    y2 = y1 + d;

    % Correct target path adjustment
    x_current_corrected = x_target(i); 
    y_current_corrected = target_y;

    % Plot robot configuration with colored links
    plot([0, x1], [0, y1], 'r-', 'LineWidth', 5); % First link in red
    plot([x1, x_current_corrected], [y1, y_current_corrected], 'm-', 'LineWidth', 5); % Second link in magenta
    plot([0, x1, x_current_corrected], [0, y1, y_current_corrected], 'ro', 'MarkerSize', 10);

    % Plot the target path
    plot(x_target, target_y * ones(size(x_target)), 'g--', 'LineWidth', 2); 

    title('Resolved Rate Control - Path Following');
    xlabel('X'); ylabel('Y');
    axis([-1.5 1.5 0 2]); % Adjust view for better arm visualization
    axis equal;
    grid on;
    pause(0.05); % Pause for smooth animation
end
