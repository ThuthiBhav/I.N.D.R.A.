%% =========================================================
% STEP 15 - BEHAVIOR DECISION + STABLE ROUTE PLANNING
%
% Behavior:
%   CRUISE
%   AVOID
%   EMERGENCY BRAKING
%   GOAL REACHED
%
% Main improvement:
% Vehicle follows straight goal route by default.
% Avoidance is activated ONLY when the normal route
% is actually blocked or threatened.
%
% Technologies:
% Sensor Fusion
% TTC
% Risk Assessment
% Goal-Aware Planning
% Kinematic Bicycle Model
% Pure Pursuit Steering
% Dynamic Replanning
% ==========================================================

clc;
clear;
close all;

%% =========================================================
% SIMULATION SETTINGS
% ==========================================================

simulationTime = 30;
dt = 0.05;

%% =========================================================
% VEHICLE INITIAL STATE
% ==========================================================

vehicleX = 0;
vehicleY = 0;
vehicleYaw = 0;

vehicleSpeed = 2.5;

cruiseSpeed = 2.5;
maxSpeed = 3.5;

%% =========================================================
% DESTINATION
% ==========================================================

goalX = 30;
goalY = 0;

goalTolerance = 0.8;

%% =========================================================
% KINEMATIC BICYCLE MODEL
% ==========================================================

wheelbase = 2.5;

maxSteeringAngle = deg2rad(30);

accelerationRate = 0.5;

brakingRate = 3.0;

%% =========================================================
% VEHICLE SAFETY
% ==========================================================

vehicleHalfWidth = 1.0;

safetyDistance = 2.0;

safetyMargin = 0.5;

minimumSafeSeparation = ...
    safetyDistance + ...
    safetyMargin;

%% =========================================================
% OBSTACLE DETECTION
% ==========================================================

forwardDetectionRange = 15;

% Only obstacles inside this corridor
% can trigger avoidance.

routeCorridor = ...
    vehicleHalfWidth + safetyDistance;

%% =========================================================
% TTC
% ==========================================================

warningTTC = 5.0;

dangerTTC = 1.5;

%% =========================================================
% PATH PLANNING
% ==========================================================

replanInterval = 0.5;

lastReplanTime = -inf;
R
lookAheadDistance = 3.5;

% Side avoidance options
avoidanceOffsets = [-4 -2 2 4];

%% =========================================================
% ROAD USERS
%
% [X Y Vx Vy]
% ==========================================================

objects = [

    12.0   0.0   -0.20   0.00;     % Car - blocks route

    20.0   7.0   -0.10   0.00;     % Auto - outside route

    23.0  -7.0    0.00   0.00;     % Pedestrian - outside route

    26.0   4.0   -0.10   0.00      % Cattle - outside route

];

objectNames = {

    'Car'
    'Auto'
    'Pedestrian'
    'Cattle'

};

%% =========================================================
% OBJECT PRIORITY
% ==========================================================

objectPriority = [

    1.0       % Car
    1.2       % Auto
    2.0       % Pedestrian
    2.5       % Cattle

];

numObjects = size(objects,1);

%% =========================================================
% SENSOR NOISE
% ==========================================================

cameraNoise = 0.10;

lidarNoise = 0.05;

radarNoise = 0.03;

%% =========================================================
% FIGURE
% ==========================================================

figure('Color','w');

hold on;
grid on;
axis equal;

xlim([-5 35]);
ylim([-10 10]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Step 15 - Stable Adaptive Autonomous Driving');

%% =========================================================
% ROAD BOUNDARIES
% ==========================================================

plot( ...
    [-10 40], ...
    [6 6], ...
    'k--', ...
    'LineWidth',1);

plot( ...
    [-10 40], ...
    [-6 -6], ...
    'k--', ...
    'LineWidth',1);

%% =========================================================
% GOAL
% ==========================================================

plot( ...
    goalX, ...
    goalY, ...
    'gp', ...
    'MarkerSize',16, ...
    'MarkerFaceColor','g');

text( ...
    goalX+0.5, ...
    goalY+0.5, ...
    'DESTINATION', ...
    'FontSize',10);

%% =========================================================
% OBJECT GRAPHICS
% ==========================================================

objectPlots = gobjects(numObjects,1);

for i = 1:numObjects

    objectPlots(i) = plot( ...
        objects(i,1), ...
        objects(i,2), ...
        'ro', ...
        'MarkerSize',10, ...
        'MarkerFaceColor','r');

end

%% =========================================================
% VEHICLE GRAPHICS
% ==========================================================

vehicleLength = 4.5;

vehicleWidth = 2.0;

halfLength = vehicleLength/2;

halfWidth = vehicleWidth/2;

vehicleBody = patch( ...
    'XData',[], ...
    'YData',[], ...
    'FaceColor',[0.2 0.6 1.0], ...
    'EdgeColor','k', ...
    'LineWidth',2);

headingLine = plot( ...
    [0 0], ...
    [0 0], ...
    'k-', ...
    'LineWidth',2);

%% =========================================================
% STATE VARIABLES
% ==========================================================

time = 0;

vehicleState = "CRUISE";

currentSteering = 0;

selectedPath = [];

avoidanceActive = false;

minimumTTC = inf;

mostDangerousObject = "None";

highestRisk = 0;

%% =========================================================
% MAIN SIMULATION LOOP
% ==========================================================

while time <= simulationTime

    %% =====================================================
    % DISTANCE TO GOAL
    % =====================================================

    distanceToGoal = sqrt( ...
        (goalX-vehicleX)^2 + ...
        (goalY-vehicleY)^2);

    %% =====================================================
    % GOAL REACHED
    % =====================================================

    if distanceToGoal < goalTolerance

        vehicleSpeed = 0;

        currentSteering = 0;

        vehicleState = "GOAL REACHED";

        title('Step 15 - GOAL REACHED');

        drawnow;

        break;

    end

    %% =====================================================
    % SENSOR SIMULATION
    % =====================================================

    cameraMeasurements = objects;

    cameraMeasurements(:,1:2) = ...
        cameraMeasurements(:,1:2) + ...
        randn(numObjects,2)*cameraNoise;

    lidarMeasurements = objects;

    lidarMeasurements(:,1:2) = ...
        lidarMeasurements(:,1:2) + ...
        randn(numObjects,2)*lidarNoise;

    radarMeasurements = objects;

    radarMeasurements(:,3:4) = ...
        radarMeasurements(:,3:4) + ...
        randn(numObjects,2)*radarNoise;

    %% =====================================================
    % SENSOR FUSION
    % =====================================================

    fusedObjects = zeros(numObjects,4);

    fusedObjects(:,1:2) = ...
        (cameraMeasurements(:,1:2) + ...
         lidarMeasurements(:,1:2))/2;

    fusedObjects(:,3:4) = ...
        radarMeasurements(:,3:4);

    %% =====================================================
    % RISK ANALYSIS
    % =====================================================

    minimumTTC = inf;

    highestRisk = 0;

    mostDangerousObject = "None";

    routeBlocked = false;

    %% =====================================================
    % ANALYZE EACH OBJECT
    % =====================================================

    for obj = 1:numObjects

        %% -----------------------------------------------
        % Relative position
        % -----------------------------------------------

        relativeX = ...
            fusedObjects(obj,1) - vehicleX;

        relativeY = ...
            fusedObjects(obj,2) - vehicleY;

        %% -----------------------------------------------
        % Convert into vehicle coordinates
        % -----------------------------------------------

        forwardDistance = ...
            relativeX*cos(vehicleYaw) + ...
            relativeY*sin(vehicleYaw);

        lateralDistance = ...
            -relativeX*sin(vehicleYaw) + ...
             relativeY*cos(vehicleYaw);

        %% -----------------------------------------------
        % Ignore objects behind
        % -----------------------------------------------

        if forwardDistance <= 0

            continue;

        end

        %% -----------------------------------------------
        % Ignore objects too far
        % -----------------------------------------------

        if forwardDistance > ...
                forwardDetectionRange

            continue;

        end

        %% -----------------------------------------------
        % Distance
        % -----------------------------------------------

        distance = sqrt( ...
            relativeX^2 + ...
            relativeY^2);

        %% -----------------------------------------------
        % Relative velocity
        % -----------------------------------------------

        relativeVX = ...
            fusedObjects(obj,3) - ...
            vehicleSpeed*cos(vehicleYaw);

        relativeVY = ...
            fusedObjects(obj,4) - ...
            vehicleSpeed*sin(vehicleYaw);

        %% -----------------------------------------------
        % Closing speed
        % -----------------------------------------------

        closingSpeed = - ...
            (relativeX*relativeVX + ...
             relativeY*relativeVY) / ...
            max(distance,0.001);

        %% -----------------------------------------------
        % TTC
        % -----------------------------------------------

        if closingSpeed > 0

            TTC = ...
                distance/closingSpeed;

        else

            TTC = inf;

        end

        %% -----------------------------------------------
        % Minimum TTC
        % -----------------------------------------------

        if TTC < minimumTTC

            minimumTTC = TTC;

        end

        %% -----------------------------------------------
        % Is object actually blocking our route?
        % -----------------------------------------------

        if abs(lateralDistance) <= ...
                routeCorridor

            if forwardDistance <= ...
                    forwardDetectionRange

                routeBlocked = true;

            end

        end

        %% -----------------------------------------------
        % Risk
        % -----------------------------------------------

        if TTC < dangerTTC

            TTCrisk = 3;

        elseif TTC < warningTTC

            TTCrisk = 2;

        elseif TTC < 8

            TTCrisk = 1;

        else

            TTCrisk = 0;

        end

        %% Distance risk

        if distance < safetyDistance

            distanceRisk = 3;

        elseif distance < 4

            distanceRisk = 1;

        else

            distanceRisk = 0;

        end

        %% Object priority

        priorityRisk = ...
            objectPriority(obj);

        %% Total risk

        totalRisk = ...
            TTCrisk + ...
            distanceRisk + ...
            priorityRisk;

        %% Most dangerous object

        if totalRisk > highestRisk

            highestRisk = totalRisk;

            mostDangerousObject = ...
                objectNames{obj};

        end

    end

    %% =====================================================
    % BEHAVIOR DECISION
    % =====================================================
    %
    % IMPORTANT:
    %
    % No obstacle blocking route:
    %       CRUISE
    %
    % Obstacle blocking route:
    %       AVOID
    %
    % No safe avoidance path:
    %       EMERGENCY BRAKING
    %
    % =====================================================

    if ~avoidanceActive

        if routeBlocked

            avoidanceActive = true;

            vehicleState = "AVOID";

        else

            vehicleState = "CRUISE";

        end

    end

    %% =====================================================
    % PATH PLANNING
    % =====================================================

    if time-lastReplanTime >= ...
            replanInterval

        lastReplanTime = time;

        %% =================================================
        % NORMAL CRUISE PATH
        % =================================================

        if ~avoidanceActive

            numPoints = 120;

            t = linspace(0,1,numPoints);

            pathX = ...
                vehicleX + ...
                (goalX-vehicleX).*t;

            pathY = ...
                vehicleY + ...
                (goalY-vehicleY).*t;

            selectedPath = ...
                [pathX' pathY'];

        %% =================================================
        % AVOIDANCE PATH
        % =================================================

        else

            numCandidates = ...
                length(avoidanceOffsets);

            candidatePaths = ...
                cell(numCandidates,1);

            candidateCosts = ...
                inf(numCandidates,1);

            safePaths = ...
                false(numCandidates,1);

            candidateSeparations = ...
                inf(numCandidates,1);

            %% ---------------------------------------------
            % Generate candidate avoidance paths
            % ---------------------------------------------

            for p = 1:numCandidates

                offset = ...
                    avoidanceOffsets(p);

                numPoints = 120;

                t = linspace(0,1,numPoints);

                %% X toward goal

                pathX = ...
                    vehicleX + ...
                    (goalX-vehicleX).*t;

                %% Base route

                baseY = ...
                    vehicleY + ...
                    (goalY-vehicleY).*t;

                %% Smooth avoidance curve

                lateralOffset = ...
                    offset*sin(pi*t);

                pathY = ...
                    baseY + ...
                    lateralOffset;

                candidatePaths{p} = ...
                    [pathX' pathY'];

                %% =========================================
                % SAFETY CHECK
                % ==========================================

                minimumSeparation = inf;

                for obj = 1:numObjects

                    predictionHorizon = 4;

                    predictedX = ...
                        fusedObjects(obj,1) + ...
                        fusedObjects(obj,3).* ...
                        (t*predictionHorizon);

                    predictedY = ...
                        fusedObjects(obj,2) + ...
                        fusedObjects(obj,4).* ...
                        (t*predictionHorizon);

                    distances = sqrt( ...
                        (pathX-predictedX).^2 + ...
                        (pathY-predictedY).^2);

                    currentMinimum = ...
                        min(distances);

                    minimumSeparation = ...
                        min( ...
                        minimumSeparation, ...
                        currentMinimum);

                end

                candidateSeparations(p) = ...
                    minimumSeparation;

                %% =========================================
                % SAFE PATH
                % ==========================================

                if minimumSeparation >= ...
                        minimumSafeSeparation

                    safePaths(p) = true;

                    %% Path length

                    pathLength = ...
                        sum(sqrt( ...
                        diff(pathX).^2 + ...
                        diff(pathY).^2));

                    %% Smoothness

                    smoothness = ...
                        sum(abs(diff(pathY,2)));

                    %% Smaller lateral movement preferred

                    lateralCost = ...
                        abs(offset)*0.5;

                    %% Total cost

                    candidateCosts(p) = ...
                        pathLength + ...
                        smoothness + ...
                        lateralCost;

                end

            end

            %% =============================================
            % SELECT BEST AVOIDANCE PATH
            % ==============================================

            if any(safePaths)

                safeCosts = ...
                    candidateCosts;

                safeCosts(~safePaths) = inf;

                [~,bestIndex] = ...
                    min(safeCosts);

                selectedPath = ...
                    candidatePaths{bestIndex};

                vehicleState = "AVOID";

            else

                %% No safe path

                selectedPath = [];

                vehicleState = ...
                    "EMERGENCY BRAKING";

            end

        end

    end

    %% =====================================================
    % CHECK WHETHER AVOIDANCE IS FINISHED
    % =====================================================

    if avoidanceActive && ...
       ~routeBlocked

        %% Return to normal route

        avoidanceActive = false;

        vehicleState = "CRUISE";

        %% Straight route will be generated
        % during the next planning cycle.

    end

    %% =====================================================
    % VEHICLE SPEED CONTROL
    % =====================================================

    if vehicleState == "CRUISE"

        %% Normal driving

        vehicleSpeed = min( ...
            cruiseSpeed, ...
            vehicleSpeed + ...
            accelerationRate*dt);

    elseif vehicleState == "AVOID"

        %% Slightly reduce speed during avoidance

        vehicleSpeed = max( ...
            1.8, ...
            vehicleSpeed - ...
            0.15*dt);

    elseif vehicleState == ...
            "EMERGENCY BRAKING"

        %% Strong braking

        vehicleSpeed = max( ...
            0, ...
            vehicleSpeed - ...
            brakingRate*dt);

    end

    %% =====================================================
    % STEERING
    % =====================================================

    if ~isempty(selectedPath) && ...
       vehicleState ~= ...
       "EMERGENCY BRAKING"

        %% -----------------------------------------------
        % Find look-ahead point
        % -----------------------------------------------

        distances = sqrt( ...
            (selectedPath(:,1)-vehicleX).^2 + ...
            (selectedPath(:,2)-vehicleY).^2);

        validPoints = find( ...
            distances >= ...
            lookAheadDistance);

        if isempty(validPoints)

            targetIndex = ...
                size(selectedPath,1);

        else

            targetIndex = ...
                validPoints(1);

        end

        %% Target

        targetX = ...
            selectedPath(targetIndex,1);

        targetY = ...
            selectedPath(targetIndex,2);

        %% Desired heading

        desiredHeading = atan2( ...
            targetY-vehicleY, ...
            targetX-vehicleX);

        %% Heading error

        headingError = ...
            desiredHeading-vehicleYaw;

        headingError = atan2( ...
            sin(headingError), ...
            cos(headingError));

        %% Pure pursuit

        currentSteering = atan2( ...
            2*wheelbase*sin(headingError), ...
            lookAheadDistance);

    else

        currentSteering = 0;

    end

    %% =====================================================
    % LIMIT STEERING
    % =====================================================

    currentSteering = max( ...
        -maxSteeringAngle, ...
        min(maxSteeringAngle, ...
        currentSteering));

    %% =====================================================
    % KINEMATIC BICYCLE MODEL
    % =====================================================

    vehicleX_dot = ...
        vehicleSpeed*cos(vehicleYaw);

    vehicleY_dot = ...
        vehicleSpeed*sin(vehicleYaw);

    vehicleYaw_dot = ...
        (vehicleSpeed/wheelbase)* ...
        tan(currentSteering);

    %% =====================================================
    % UPDATE VEHICLE
    % =====================================================

    vehicleX = ...
        vehicleX + ...
        vehicleX_dot*dt;

    vehicleY = ...
        vehicleY + ...
        vehicleY_dot*dt;

    vehicleYaw = ...
        vehicleYaw + ...
        vehicleYaw_dot*dt;

    %% =====================================================
    % UPDATE ROAD USERS
    % =====================================================

    objects(:,1) = ...
        objects(:,1) + ...
        objects(:,3)*dt;

    objects(:,2) = ...
        objects(:,2) + ...
        objects(:,4)*dt;

    %% =====================================================
    % DRAW PATHS
    % =====================================================

    delete(findobj(gca,'Tag','candidatePath'));

    delete(findobj(gca,'Tag','selectedPath'));

    %% =====================================================
    % DRAW AVOIDANCE CANDIDATES
    % =====================================================

    if avoidanceActive

        for p = 1:numCandidates

            path = candidatePaths{p};

            if safePaths(p)

                plot( ...
                    path(:,1), ...
                    path(:,2), ...
                    '--', ...
                    'LineWidth',1, ...
                    'Tag','candidatePath');

            else

                plot( ...
                    path(:,1), ...
                    path(:,2), ...
                    ':', ...
                    'LineWidth',1, ...
                    'Tag','candidatePath');

            end

        end

    end

    %% =====================================================
    % DRAW SELECTED PATH
    % =====================================================

    if ~isempty(selectedPath)

        plot( ...
            selectedPath(:,1), ...
            selectedPath(:,2), ...
            'k-', ...
            'LineWidth',2.5, ...
            'Tag','selectedPath');

    end

    %% =====================================================
    % DRAW VEHICLE
    % =====================================================

    localCorners = [

         halfLength   halfWidth
         halfLength  -halfWidth
        -halfLength  -halfWidth
        -halfLength   halfWidth

    ];

    rotationMatrix = [

        cos(vehicleYaw) -sin(vehicleYaw)
        sin(vehicleYaw)  cos(vehicleYaw)

    ];

    rotatedCorners = ...
        localCorners*rotationMatrix';

    rotatedCorners(:,1) = ...
        rotatedCorners(:,1)+vehicleX;

    rotatedCorners(:,2) = ...
        rotatedCorners(:,2)+vehicleY;

    set(vehicleBody, ...
        'XData',rotatedCorners(:,1), ...
        'YData',rotatedCorners(:,2));

    %% =====================================================
    % HEADING
    % =====================================================

    headingLength = 2;

    set(headingLine, ...
        'XData',[ ...
        vehicleX ...
        vehicleX + ...
        headingLength*cos(vehicleYaw)], ...
        'YData',[ ...
        vehicleY ...
        vehicleY + ...
        headingLength*sin(vehicleYaw)]);

    %% =====================================================
    % UPDATE OBJECTS
    % =====================================================

    for i = 1:numObjects

        set(objectPlots(i), ...
            'XData',objects(i,1), ...
            'YData',objects(i,2));

    end

    %% =====================================================
    % DISPLAY
    % =====================================================

    if isinf(minimumTTC)

        TTCtext = 'INF';

    else

        TTCtext = sprintf( ...
            '%.2f s',minimumTTC);

    end

    title(sprintf( ...
        'Step 15 - Behavior: %s', ...
        vehicleState));

    text( ...
        -4,8.8, ...
        sprintf( ...
        'Time: %.1f s | Speed: %.2f m/s | Steering: %.1f deg', ...
        time, ...
        vehicleSpeed, ...
        rad2deg(currentSteering)), ...
        'FontSize',10);

    text( ...
        -4,8.0, ...
        sprintf( ...
        'Goal Distance: %.2f m', ...
        distanceToGoal), ...
        'FontSize',10);

    text( ...
        -4,7.2, ...
        sprintf( ...
        'TTC: %s | Risk: %.1f', ...
        TTCtext, ...
        highestRisk), ...
        'FontSize',10);

    text( ...
        -4,6.4, ...
        sprintf( ...
        'Dangerous Object: %s', ...
        mostDangerousObject), ...
        'FontSize',10);

    drawnow;

    %% =====================================================
    % TIME
    % =====================================================

    time = time + dt;

end

%% =========================================================
% FINAL OUTPUT
% ==========================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('STEP 15 - SIMULATION COMPLETE\n');
fprintf('====================================================\n');

fprintf('Final X position       = %.2f m\n',vehicleX);

fprintf('Final Y position       = %.2f m\n',vehicleY);

fprintf('Final heading          = %.2f degrees\n', ...
    rad2deg(vehicleYaw));

fprintf('Final speed            = %.2f m/s\n', ...
    vehicleSpeed);

fprintf('Goal position          = [%.2f, %.2f]\n', ...
    goalX,goalY);

fprintf('Distance to goal       = %.2f m\n', ...
    sqrt((goalX-vehicleX)^2 + ...
         (goalY-vehicleY)^2));

fprintf('Minimum TTC            = %.2f s\n', ...
    minimumTTC);

fprintf('Most dangerous object = %s\n', ...
    mostDangerousObject);

fprintf('\n');
fprintf('Stable behavior-based autonomous driving completed.\n');