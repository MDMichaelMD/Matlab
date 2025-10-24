function J = jacob(q)
% JACOB Computes the Jacobian matrix for a planar two-link robotic manipulator.
% Input:
%   q - 2x1 vector of joint variables [theta; d]'
% Output:
%   J - 6x2 Jacobian matrix

% Extract joint variables
theta = q(1); % Rotation angle of the first joint
d = q(2);    % Displacement along the prismatic joint

% Define link length
L1 = 1; % Length of the first link in meters

% Homogeneous transformation to the first joint
o0 = [0; 0; 0]; % Base frame origin
o1 = [L1 * cos(theta); L1 * sin(theta); 0]; % Position of first joint
o2 = [L1 * cos(theta); L1 * sin(theta) + d; 0]; % End-effector position

% Define the z-axis unit vector
z0 = [0; 0; 1]; % Base frame z-axis

% Compute Translational Jacobian (Jv)
Jv1 = cross(z0, o2 - o0); % Contribution from the first joint
Jv2 = z0; % Contribution from the prismatic joint
J_v = [Jv1, Jv2];

% Compute Rotational Jacobian (Jw)
Jw1 = z0; % First joint provides rotation
Jw2 = [0; 0; 0]; % Second joint has no rotation contribution
J_w = [Jw1, Jw2];

% Combine Translational and Rotational Jacobians
J = [J_v; J_w];
end
