% File: drawrobot.m
% Description: Draws the two-link planar robot at a given configuration.

function drawrobot(q)
    theta = q(1); % Rotation angle of the first joint
    d = q(2);     % Displacement of the prismatic joint
    
    % Define base, joint, and end-effector positions
    base = [0; 0]; % Robot base point
    joint = [cos(theta); sin(theta)]; % End of the first link
    end_eff = joint + [cos(theta); sin(theta)] * d; % End-effector position
    
    % Plot the robot structure
    figure(1); clf; hold on;
    plot([base(1), joint(1)], [base(2), joint(2)], 'b-', 'LineWidth', 2); % First link
    plot([joint(1), end_eff(1)], [joint(2), end_eff(2)], 'r-', 'LineWidth', 2); % Second link
    
    % Plot the end-effector as a green circle
    plot(end_eff(1), end_eff(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 

    % Set axis and labels for clarity
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    title('Planar Robot Configuration');
end

% % Test Cases
% % Uncomment to run
% drawrobot([0; 0]); % Expect horizontal first link, no extension
% pause(2);
% drawrobot([pi/2; 0]); % Expect vertical first link, no extension
% pause(2);
% drawrobot([pi/4; 1]); % Expect 45-degree diagonal extension
% pause(2);
% drawrobot([pi/4; -0.5]); % Expect prismatic joint retraction
% pause(2);
