% File: fk.m
% Description: Computes the end-effector position for the planar robot.

function x = fk(q)
    theta = q(1); % Rotation angle of the first joint
    d = q(2);     % Displacement along the prismatic joint
    
    % Compute the end-effector position based on the first link (L1 = 1 meter)
    x = [cos(theta) * 1; sin(theta) * 1] + [0; d];
end

% %% Test Example
% % Correct Function Call Example
% q = [0; 0];    % Initial configuration [theta, d]
% x = fk(q);     % Call the forward kinematics function
% disp(x);       % Display the computed end-effector position
