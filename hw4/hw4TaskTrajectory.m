clear, close all
%addpath("robotics toolbox\")

%% Settings

% time parameters
timePerTurn = 16; % time for one full turn [s]
timeStep = 0.1; % length of one timestep [s]
timeToStart = 4; % time to move from home to start position [s]

% given parameters
numTurns = 10; % amount of turns
amplitude = 0.5; % [m]
thetaLimits = [0, 360]; % [DEG] - one rotation is actually only 0°-180°; setting this to 3600 and numTurns=1 works too (but increase timePerTurn*10)
initialJointAngles = [pi/4; pi/4; pi/4];  % [RAD]

%% Initialization

% home position
homePosition = fk(initialJointAngles);

% time
totalTime = timeToStart + numTurns * timePerTurn; % total simulation time [s]
timePerSingleTurn = 0:timeStep:timePerTurn; % time vector for one turn
completeTimeVector = 0:timeStep:totalTime; % complete time vector

% preallocate for speed
% EE pos/vel/accel
positionPerTurn = NaN(2, length(timePerSingleTurn));
% pd_turn = NaN(2,length(timePerSingleTurn));
% pdd_turn = NaN(2,length(timePerSingleTurn));
% joint values
jointAnglesPerTurn = NaN(3, length(timePerSingleTurn));

%% Task-space trajectory planning

% Generate a smooth movement inclusive of acceleration and deceleration from start to end
zeroVector = zeros(3, 1);
[thetaValues, ~, ~] = interpolTaskSpace([[0, 0, thetaLimits(1)]', zeroVector, zeroVector], ...
                                        [[0, 0, thetaLimits(2)]', zeroVector, zeroVector], ...
                                        timePerSingleTurn);
thetaValues = thetaValues(3, :); % only care about this dimension

% Calculate position based on the given equations
positionPerTurn = [amplitude .* cosd(3 .* thetaValues) .* cosd(thetaValues) + homePosition(1);
                   amplitude .* cosd(3 .* thetaValues) .* sind(thetaValues) + homePosition(2);
                   zeros(1, length(thetaValues))];

% Perform inverse kinematics to get each joint value for every timestep
for i = 1:size(positionPerTurn, 2)
    jointAnglesPerTurn(:, i) = ik(positionPerTurn(:, i));
end

% Move from the home position to the start of the trajectory
startJointAngles = ik(positionPerTurn(:, 1));
[startJointInterpolation, ~, ~] = interpolTaskSpace([initialJointAngles, zeroVector, zeroVector], ...
                                                   [startJointAngles', zeroVector, zeroVector], ...
                                                   0:timeStep:timeToStart);

% Combine trajectory joint values for all turns
jointAnglesOverTime = [startJointInterpolation, repmat(jointAnglesPerTurn, 1, numTurns)];

%% Animation
% Animate the movement of the robot

% Initialize the figure
figure('units', 'normalized', 'outerposition', [0 0 1 1]);
hold on
axis equal
grid on
xlim([-1 1])
ylim([0 3])

% Plot the goal trajectory
plot(positionPerTurn(1, :), positionPerTurn(2, :), 'b');

% Plot the robot as a line
robotLinePlot = drawrobot(jointAnglesOverTime(:, 1));

% Main animation loop
for frame = 2:length(completeTimeVector)
    % Update the robot representation
    drawrobot(jointAnglesOverTime(:, frame), robotLinePlot);
    
    % Force MATLAB to update the figure 
    drawnow
    pause(timeStep)
end
