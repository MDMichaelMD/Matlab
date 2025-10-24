clear, close all

%% Settings
% Simulation and robot parameters
numTurns = 10;             % Number of revolutions
amplitude = 0.5;           % [m] Amplitude of the trajectory
thetaDeg = 0:2:numTurns*360; % [deg] Angular steps in degrees
initialJointAngles = [pi/4; pi/4; pi/4]; % [rad] Initial joint angles

% Resolved Rate Algorithm parameters
maxIterations = 1000;      % Maximum iterations
timeStep = 0.01;           % [s] Time step for the algorithm
posTolerance = 0.001;      % [m] Position error tolerance (1mm)
endEffectorSpeed = 0.1;    % [m/s] Desired end effector speed

% Animation parameters
animationTimeStep = 0.2;   % [s] Time step for animation frames

%% Initialization
% Storage for end-effector positions and joint values
endEffectorPositions = NaN(3, length(thetaDeg));
jointAnglesOverTheta = NaN(3, length(thetaDeg));

%% Resolved Rate Algorithm
% Initial position
initialPosition = fk(initialJointAngles);

% Desired trajectory
trajectory = [amplitude .* cosd(3 .* thetaDeg) .* cosd(thetaDeg) + initialPosition(1);
              amplitude .* cosd(3 .* thetaDeg) .* sind(thetaDeg) + initialPosition(2);
              zeros(1, length(thetaDeg))];

% Initial joint configuration
currentJointAngles = initialJointAngles;

% Store all joint angles over time
jointAnglesHistory = [];

% Main loop over trajectory points
for stepIdx = 1:size(trajectory, 2)
    desiredPosition = trajectory(:, stepIdx); % Current target position
    exitFlag = false; % Convergence flag
    singularityDetected = false;

    for iterIdx = 1:maxIterations + 1
        % Current end-effector position
        currentPosition = fk(currentJointAngles);

        % Position error
        positionError = desiredPosition - currentPosition;
        errorNorm = norm(positionError);

        % Check for convergence or max iterations
        if errorNorm < posTolerance
            break;
        elseif iterIdx == maxIterations + 1
            fprintf('Step %d: Max iterations exceeded\n', stepIdx);
            break;
        end

        % Desired end-effector velocity
        desiredVelocity = endEffectorSpeed * positionError / errorNorm;

        % Jacobian computation
        jacobianMatrix = buildJacobian(currentJointAngles);
        jacobianMatrix = jacobianMatrix(1:3, :); % Use only positional part

        % Handle singularities
        if min(svd(jacobianMatrix)) < 1e-3
            singularityDetected = true;
            pseudoInverseJacobian = jacobianMatrix' / (jacobianMatrix * jacobianMatrix' + 1e-3 * eye(size(jacobianMatrix, 1)));
        else
            singularityDetected = false;
            pseudoInverseJacobian = pinv(jacobianMatrix);
        end

        % Compute joint velocity
        jointVelocity = pseudoInverseJacobian * desiredVelocity;

        % Update joint angles
        currentJointAngles = currentJointAngles + jointVelocity * timeStep;

        % Store joint angles for animation
        jointAnglesHistory = [jointAnglesHistory, currentJointAngles];
    end

    % Store results
    endEffectorPositions(:, stepIdx) = currentPosition;
    jointAnglesOverTheta(:, stepIdx) = currentJointAngles;
end

%% Plots
% Plot joint angles over the trajectory
figure;
hold on;
plot(thetaDeg, jointAnglesOverTheta(1, :), 'DisplayName', 'q1');
plot(thetaDeg, jointAnglesOverTheta(2, :), 'DisplayName', 'q2');
plot(thetaDeg, jointAnglesOverTheta(3, :), 'DisplayName', 'q3');
legend;
title('Joint Angles over Trajectory');
xlabel('\theta [deg]');
ylabel('Joint Angles [rad]');
grid on;

%% Animation
% Downsample data for animation
stepSize = round(animationTimeStep / timeStep);
animationJointAngles = jointAnglesHistory(:, 1:stepSize:end);

% Initialize animation
figure('units', 'normalized', 'outerposition', [0 0 1 1]);
hold on;
axis equal;
grid on;
xlim([-0.6 1.2]);
ylim([0 3]);

% Plot trajectory
plot(trajectory(1, :), trajectory(2, :), 'b');

% Initialize robot plot
robotPlot = drawrobot(animationJointAngles(:, 1));

% Animation loop
for frameIdx = 2:length(animationJointAngles)
    % Update robot representation
    drawrobot(animationJointAngles(:, frameIdx), robotPlot);
    drawnow;
end
