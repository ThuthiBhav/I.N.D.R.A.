%% ================================================================
%  SIH26037 - ADAPTIVE PATH PLANNING & COLLISION AVOIDANCE
%  FINAL SIH DEMONSTRATION
%
%  Scenario:
%  Sudden Cattle Crossing on an Unstructured Indian Road
%
%  Pipeline:
%  Sensor Simulation
%       -> Sensor Fusion
%       -> Prediction
%       -> TTC / Risk
%       -> Decision
%       -> Adaptive Path Planning
%       -> Vehicle Control
%       -> Vehicle Motion
%
%  This is a MATLAB software-in-the-loop prototype.
% ================================================================

clear;
clc;
close all;

%% ===================== SCENARIO SETTINGS =========================

simulationTime = 18;
dt = 0.05;
time = 0:dt:simulationTime;

% Road
roadWidth = 8;
goalX = 45;
goalY = 0;

% Ego vehicle
egoX = 0;
egoY = 0;
egoYaw = 0;
egoSpeed = 3.5;

vehicleLength = 3.8;
vehicleWidth = 1.8;

% Safety
safetyDistance = 2.5;
safetyMargin = 1.0;
minimumSafeDistance = safetyDistance + safetyMargin;

% Sensor ranges
cameraRange = 25;
lidarRange = 30;
radarRange = 35;

% TTC thresholds
warningTTC = 5.0;
dangerTTC = 1.5;

% Planning
replanningInterval = 0.5;
lastReplanTime = -inf;

candidateOffsets = [-3.5 -2 0 2 3.5];

%% ===================== METRICS ================================

replanningCount = 0;
collisionDetected = false;
goalReached = false;

minimumDistance = inf;
maximumRisk = 0;

speedHistory = [];
riskHistory = [];
distanceHistory = [];

previousMode = "CRUISE";

%% ===================== FIGURE ================================

fig = figure( ...
    'Name','SIH26037 - Adaptive Autonomous Driving Demo', ...
    'NumberTitle','off', ...
    'Color','w');

set(fig,'Position',[80 80 1400 780]);

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');

xlim(ax,[-5 goalX+5]);
ylim(ax,[-10 10]);

xlabel(ax,'Longitudinal Position (m)');
ylabel(ax,'Lateral Position (m)');

title(ax, ...
    'SIH26037 | Adaptive Path Planning and Collision Avoidance', ...
    'FontSize',16);

%% ===================== ROAD ====================================

plot(ax,[-5 goalX+5], ...
    [roadWidth/2 roadWidth/2], ...
    'k--','LineWidth',1.5);

plot(ax,[-5 goalX+5], ...
    [-roadWidth/2 -roadWidth/2], ...
    'k--','LineWidth',1.5);

% No lane markings intentionally
text(ax,5,7.2,...
    'UNSTRUCTURED INDIAN ROAD | NO LANE MARKINGS',...
    'FontSize',11);

% Goal
plot(ax,goalX,goalY,'p',...
    'MarkerSize',16,...
    'LineWidth',2);

text(ax,goalX-1.5,1.2,'GOAL',...
    'FontWeight','bold');

%% ===================== VEHICLE GRAPHICS =========================

egoPlot = plot(ax,egoX,egoY,'s',...
    'MarkerSize',11,...
    'LineWidth',2);

plannedPathPlot = plot(ax,nan,nan,...
    'LineWidth',2);

predictionPlot = plot(ax,nan,nan,...
    '--','LineWidth',1.5);

%% ===================== OBJECT GRAPHICS ==========================

% Cattle
cattlePlot = plot(ax,nan,nan,'o',...
    'MarkerSize',12,...
    'LineWidth',2);

% Other road users
carPlot = plot(ax,nan,nan,'s',...
    'MarkerSize',9,...
    'LineWidth',1.5);

autoPlot = plot(ax,nan,nan,'d',...
    'MarkerSize',9,...
    'LineWidth',1.5);

pedPlot = plot(ax,nan,nan,'^',...
    'MarkerSize',9,...
    'LineWidth',1.5);

%% ===================== HUD =====================================

hud = annotation(fig,'textbox',...
    [0.70 0.55 0.27 0.38],...
    'String','',...
    'FitBoxToText','off',...
    'BackgroundColor','w',...
    'EdgeColor','k',...
    'FontSize',11);

statusText = annotation(fig,'textbox',...
    [0.70 0.45 0.27 0.07],...
    'String','INITIALIZING...',...
    'FitBoxToText','off',...
    'HorizontalAlignment','center',...
    'FontSize',15,...
    'FontWeight','bold');

%% ===================== MAIN SIMULATION ==========================

for k = 1:length(time)

    t = time(k);

    %% ------------------------------------------------------------
    % 1. ROAD USERS / ENVIRONMENT
    % ------------------------------------------------------------

    % Slow vehicle ahead
    carX = 17 - 0.10*t;
    carY = 0.8;
    carVx = -0.10;
    carVy = 0;

    % Auto on roadside
    autoX = 25 - 0.05*t;
    autoY = 5.0;
    autoVx = -0.05;
    autoVy = 0;

    % Pedestrian
    pedX = 29;
    pedY = -6 + 0.10*t;
    pedVx = 0;
    pedVy = 0.10;

    %% ------------------------------------------------------------
    % 2. SUDDEN CATTLE CROSSING
    % ------------------------------------------------------------

    % Before t=5 cattle remains near roadside.
    % After t=5 it suddenly crosses the vehicle path.

    if t < 5

        cattleX = 19;
        cattleY = 5.5;

        cattleVx = 0;
        cattleVy = 0;

    else

        cattleX = 19 - 0.15*(t-5);
        cattleY = 5.5 - 0.75*(t-5);

        cattleVx = -0.15;
        cattleVy = -0.75;

    end

    %% ------------------------------------------------------------
    % 3. GROUND TRUTH OBJECT MATRIX
    % [x y vx vy]
    % ------------------------------------------------------------

    objects = [
        carX    carY    carVx    carVy;
        autoX   autoY   autoVx   autoVy;
        pedX    pedY    pedVx    pedVy;
        cattleX cattleY cattleVx cattleVy
    ];

    %% ------------------------------------------------------------
    % 4. SIMULATED CAMERA / LIDAR / RADAR
    % ------------------------------------------------------------

    cameraNoise = 0.25;
    lidarNoise = 0.10;
    radarVelocityNoise = 0.08;

    cameraMeasurements = objects;

    cameraMeasurements(:,1:2) = ...
        objects(:,1:2) + ...
        cameraNoise*randn(size(objects(:,1:2)));

    lidarMeasurements = objects;

    lidarMeasurements(:,1:2) = ...
        objects(:,1:2) + ...
        lidarNoise*randn(size(objects(:,1:2)));

    radarMeasurements = objects;

    radarMeasurements(:,3:4) = ...
        objects(:,3:4) + ...
        radarVelocityNoise*randn(size(objects(:,3:4)));

    %% ------------------------------------------------------------
    % 5. SENSOR FUSION
    % ------------------------------------------------------------

    fusedObjects = zeros(size(objects));

    fusedObjects(:,1:2) = ...
        0.5*cameraMeasurements(:,1:2) + ...
        0.5*lidarMeasurements(:,1:2);

    fusedObjects(:,3:4) = ...
        radarMeasurements(:,3:4);

    %% ------------------------------------------------------------
    % 6. MULTI-STEP PREDICTION
    % ------------------------------------------------------------

    predictionHorizon = 3;

    predictedCattleX = zeros(1,predictionHorizon);
    predictedCattleY = zeros(1,predictionHorizon);

    for p = 1:predictionHorizon

        predictedCattleX(p) = ...
            fusedObjects(4,1) + ...
            fusedObjects(4,3)*p;

        predictedCattleY(p) = ...
            fusedObjects(4,2) + ...
            fusedObjects(4,4)*p;

    end

    %% ------------------------------------------------------------
    % 7. TTC + RISK ANALYSIS
    % ------------------------------------------------------------

    TTC = inf;
    routeBlocked = false;
    cautionCondition = false;

    currentMinimumDistance = inf;

    objectRiskValues = zeros(4,1);

    for i = 1:4

        dx = fusedObjects(i,1) - egoX;
        dy = fusedObjects(i,2) - egoY;

        distance = sqrt(dx^2 + dy^2);

        currentMinimumDistance = ...
            min(currentMinimumDistance,distance);

        % Object relative position
        longitudinalDistance = ...
            dx*cos(egoYaw) + dy*sin(egoYaw);

        lateralDistance = ...
            -dx*sin(egoYaw) + dy*cos(egoYaw);

        % Only objects ahead
        objectAhead = longitudinalDistance > 0;

        if objectAhead

            objectForwardVelocity = ...
                fusedObjects(i,3)*cos(egoYaw) + ...
                fusedObjects(i,4)*sin(egoYaw);

            relativeSpeed = egoSpeed - objectForwardVelocity;

            if relativeSpeed > 0.05

                currentTTC = distance / relativeSpeed;

                TTC = min(TTC,currentTTC);

            end

            % Route corridor
            if longitudinalDistance < 15 && ...
                    abs(lateralDistance) < 3

                routeBlocked = true;

            end

        end

        %% Risk

        distanceRisk = 0;

        if distance < 8
            distanceRisk = (8-distance)/8;
        end

        ttcRisk = 0;

        if isfinite(TTC) && TTC < warningTTC

            ttcRisk = ...
                (warningTTC-TTC)/warningTTC;

        end

        objectRiskValues(i) = ...
            0.6*distanceRisk + ...
            0.4*ttcRisk;

    end

    riskScore = max(objectRiskValues);

    maximumRisk = max(maximumRisk,riskScore);

    %% ------------------------------------------------------------
    % 8. UNCERTAINTY-AWARE RISK
    % ------------------------------------------------------------

    % Cattle has higher uncertainty because its movement is
    % less predictable than a lane-following vehicle.

    cattleUncertainty = 2.0;

    if routeBlocked

        uncertaintyRisk = ...
            min(cattleUncertainty/5,0.4);

        riskScore = ...
            min(1,riskScore + 0.2*uncertaintyRisk);

    end

    %% ------------------------------------------------------------
    % 9. BEHAVIOR DECISION
    % ------------------------------------------------------------

    if t > 17 || ...
            sqrt((goalX-egoX)^2 + ...
                 (goalY-egoY)^2) < 1

        behavior = "GOAL REACHED";

    elseif TTC < dangerTTC

        behavior = "EMERGENCY";

    elseif routeBlocked

        behavior = "AVOID";

    elseif TTC < warningTTC || riskScore > 0.35

        behavior = "CAUTION";

    else

        behavior = "CRUISE";

    end

    %% ------------------------------------------------------------
    % 10. ADAPTIVE PATH PLANNING
    % ------------------------------------------------------------

    % Default nominal path
    selectedOffset = 0;

    selectedCost = inf;

    selectedX = [];
    selectedY = [];

    candidateForward = linspace(0,15,31);

    if behavior == "EMERGENCY"

        selectedX = egoX*ones(size(candidateForward));
        selectedY = egoY*ones(size(candidateForward));

        selectedCost = 999;

    elseif behavior == "AVOID"

        for c = 1:length(candidateOffsets)

            offset = candidateOffsets(c);

            candidateX = ...
                egoX + candidateForward;

            candidateY = ...
                egoY + ...
                offset*sin(pi*candidateForward/15);

            minimumCandidateDistance = inf;

            %% Check candidate against all predicted objects

            for i = 1:4

                for p = 1:length(candidateForward)

                    futureTime = ...
                        candidateForward(p)/max(egoSpeed,0.1);

                    predictedX = ...
                        fusedObjects(i,1) + ...
                        fusedObjects(i,3)*futureTime;

                    predictedY = ...
                        fusedObjects(i,2) + ...
                        fusedObjects(i,4)*futureTime;

                    d = sqrt( ...
                        (candidateX(p)-predictedX)^2 + ...
                        (candidateY(p)-predictedY)^2);

                    minimumCandidateDistance = ...
                        min(minimumCandidateDistance,d);

                end

            end

            %% Safety constraint

            if minimumCandidateDistance >= minimumSafeDistance

                pathLength = 0;

                for p = 2:length(candidateX)

                    pathLength = pathLength + ...
                        sqrt( ...
                        (candidateX(p)-candidateX(p-1))^2 + ...
                        (candidateY(p)-candidateY(p-1))^2);

                end

                smoothnessCost = ...
                    sum(abs(diff(candidateY,2)));

                riskPenalty = ...
                    max(0,minimumSafeDistance- ...
                    minimumCandidateDistance)*100;

                totalCost = ...
                    pathLength + ...
                    0.5*smoothnessCost + ...
                    riskPenalty + ...
                    0.2*abs(offset);

                if totalCost < selectedCost

                    selectedCost = totalCost;

                    selectedX = candidateX;
                    selectedY = candidateY;

                    selectedOffset = offset;

                end

            end

        end

        %% No safe path
        if isempty(selectedX)

            behavior = "EMERGENCY";

            selectedX = egoX*ones(size(candidateForward));
            selectedY = egoY*ones(size(candidateForward));

            selectedCost = 999;

        else

            replanningCount = replanningCount + 1;

        end

    else

        %% Normal goal-directed route

        selectedX = ...
            linspace(egoX,goalX,31);

        selectedY = ...
            linspace(egoY,goalY,31);

        selectedCost = ...
            sqrt((goalX-egoX)^2 + ...
                 (goalY-egoY)^2);

    end

    %% ------------------------------------------------------------
    % 11. SPEED CONTROL
    % ------------------------------------------------------------

    targetSpeed = 3.5;

    if behavior == "CAUTION"

        targetSpeed = 2.2;

    elseif behavior == "AVOID"

        targetSpeed = 1.8;

    elseif behavior == "EMERGENCY" || ...
            behavior == "GOAL REACHED"

        targetSpeed = 0;

    end

    %% ------------------------------------------------------------
    % 12. PURE PURSUIT STYLE STEERING
    % ------------------------------------------------------------

    if ~isempty(selectedX)

        lookAheadDistance = 3;

        distances = sqrt( ...
            (selectedX-egoX).^2 + ...
            (selectedY-egoY).^2);

        [~,closestIndex] = min(distances);

        lookAheadIndex = closestIndex;

        for j = closestIndex:length(selectedX)

            d = sqrt( ...
                (selectedX(j)-egoX)^2 + ...
                (selectedY(j)-egoY)^2);

            if d >= lookAheadDistance

                lookAheadIndex = j;
                break;

            end

        end

        targetX = selectedX(lookAheadIndex);
        targetY = selectedY(lookAheadIndex);

        dx = targetX-egoX;
        dy = targetY-egoY;

        localX = ...
            cos(egoYaw)*dx + ...
            sin(egoYaw)*dy;

        localY = ...
            -sin(egoYaw)*dx + ...
            cos(egoYaw)*dy;

        distance = sqrt(localX^2 + localY^2);

        if distance > 0.1

            curvature = ...
                2*localY/(distance^2);

        else

            curvature = 0;

        end

        wheelbase = 2.5;

        steeringAngle = ...
            atan(wheelbase*curvature);

        maximumSteering = ...
            30*pi/180;

        steeringAngle = ...
            max(-maximumSteering,...
            min(maximumSteering,steeringAngle));

    else

        steeringAngle = 0;

    end

    %% ------------------------------------------------------------
    % 13. VEHICLE DYNAMICS
    % ------------------------------------------------------------

    if egoSpeed < targetSpeed

        egoSpeed = egoSpeed + 0.8*dt;

    elseif egoSpeed > targetSpeed

        egoSpeed = egoSpeed - 1.5*dt;

    end

    egoSpeed = max(0,min(3.8,egoSpeed));

    wheelbase = 2.5;

    egoX = egoX + ...
        egoSpeed*cos(egoYaw)*dt;

    egoY = egoY + ...
        egoSpeed*sin(egoYaw)*dt;

    egoYaw = egoYaw + ...
        egoSpeed/wheelbase * ...
        tan(steeringAngle)*dt;

    %% ------------------------------------------------------------
    % 14. COLLISION CHECK
    % ------------------------------------------------------------

    for i = 1:4

        d = sqrt( ...
            (egoX-objects(i,1))^2 + ...
            (egoY-objects(i,2))^2);

        minimumDistance = min(minimumDistance,d);

        if d < 2.0

            collisionDetected = true;

        end

    end

    %% ------------------------------------------------------------
    % 15. GOAL CHECK
    % ------------------------------------------------------------

    goalDistance = sqrt( ...
        (goalX-egoX)^2 + ...
        (goalY-egoY)^2);

    if goalDistance < 1.0

        goalReached = true;

    end

    %% ------------------------------------------------------------
    % 16. STORE METRICS
    % ------------------------------------------------------------

    speedHistory(end+1) = egoSpeed;
    riskHistory(end+1) = riskScore;
    distanceHistory(end+1) = currentMinimumDistance;

    %% ------------------------------------------------------------
    % 17. UPDATE GRAPHICS
    % ------------------------------------------------------------

    set(egoPlot,...
        'XData',egoX,...
        'YData',egoY);

    set(plannedPathPlot,...
        'XData',selectedX,...
        'YData',selectedY);

    set(predictionPlot,...
        'XData',predictedCattleX,...
        'YData',predictedCattleY);

    set(cattlePlot,...
        'XData',cattleX,...
        'YData',cattleY);

    set(carPlot,...
        'XData',carX,...
        'YData',carY);

    set(autoPlot,...
        'XData',autoX,...
        'YData',autoY);

    set(pedPlot,...
        'XData',pedX,...
        'YData',pedY);

    %% ------------------------------------------------------------
    % 18. HUD UPDATE
    % ------------------------------------------------------------

    if isinf(TTC)

        ttcDisplay = "INF";

    else

        ttcDisplay = sprintf("%.2f s",TTC);

    end

    hud.String = sprintf([ ...
        'SIH26037 SYSTEM STATUS\n\n' ...
        'Scenario: Sudden Cattle Crossing\n\n' ...
        'Behavior Mode: %s\n' ...
        'Vehicle Speed: %.2f m/s\n' ...
        'TTC: %s\n' ...
        'Risk Score: %.2f\n' ...
        'Minimum Distance: %.2f m\n' ...
        'Selected Offset: %.2f m\n' ...
        'Replanning Events: %d\n\n' ...
        'PERCEPTION\n' ...
        'Camera + LiDAR + Radar\n\n' ...
        'PREDICTION\n' ...
        '3-second trajectory prediction\n\n' ...
        'PLANNING\n' ...
        'Safety-constrained candidate selection'], ...
        behavior,...
        egoSpeed,...
        ttcDisplay,...
        riskScore,...
        minimumDistance,...
        selectedOffset,...
        replanningCount);

    %% Status message

    if behavior == "CRUISE"

        statusText.String = ...
            "CRUISE  |  NOMINAL ROUTE";

    elseif behavior == "CAUTION"

        statusText.String = ...
            "CAUTION  |  RISK DETECTED";

    elseif behavior == "AVOID"

        statusText.String = ...
            "AVOID  |  ADAPTIVE REPLANNING";

    elseif behavior == "EMERGENCY"

        statusText.String = ...
            "EMERGENCY  |  SAFE STOP";

    else

        statusText.String = ...
            "GOAL REACHED";

    end

    %% ------------------------------------------------------------
    % 19. CAMERA FOLLOW
    % ------------------------------------------------------------

    xlim(ax,[max(-5,egoX-8) min(goalX+5,egoX+35)]);

    %% ------------------------------------------------------------
    % 20. LIVE UPDATE
    % ------------------------------------------------------------

    drawnow;

    pause(0.01);

end

%% ================================================================
% FINAL RESULTS
% ================================================================

scenarioCompletion = goalReached;

collisionFree = ~collisionDetected;

averageSpeed = mean(speedHistory);

averageRisk = mean(riskHistory);

if isempty(distanceHistory)

    minimumDistance = NaN;

end

%% ===================== FINAL RESULT WINDOW ======================

fprintf('\n');
fprintf('=====================================================\n');
fprintf('        SIH26037 FINAL SIMULATION RESULTS\n');
fprintf('=====================================================\n');

fprintf('Scenario              : Sudden Cattle Crossing\n');

fprintf('Collision-Free        : %d\n',collisionFree);

fprintf('Goal Reached          : %d\n',scenarioCompletion);

fprintf('Minimum Distance      : %.2f m\n',minimumDistance);

fprintf('Average Speed         : %.2f m/s\n',averageSpeed);

fprintf('Maximum Risk          : %.2f\n',maximumRisk);

fprintf('Replanning Events     : %d\n',replanningCount);

fprintf('Final Position        : (%.2f, %.2f)\n',egoX,egoY);

fprintf('=====================================================\n');

%% ===================== METRICS FIGURE ===========================

figure( ...
    'Name','SIH26037 Performance Metrics',...
    'NumberTitle','off');

tiledlayout(2,2);

nexttile;

plot(time,speedHistory,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Speed (m/s)');

title('Vehicle Speed');

nexttile;

plot(time,riskHistory,'LineWidth',2);

grid on;

xlabel('Time (s)');
ylabel('Risk Score');

title('Collision Risk');

nexttile;

plot(time,distanceHistory,'LineWidth',2);

hold on;

yline(minimumSafeDistance,'--','Safety Threshold');

grid on;

xlabel('Time (s)');
ylabel('Distance (m)');

title('Minimum Object Distance');

nexttile;

bar([ ...
    double(collisionFree),...
    double(scenarioCompletion),...
    replanningCount/100]);

grid on;

xticklabels({ ...
    'Collision Free',...
    'Goal Reached',...
    'Replanning /100'});

title('Final Performance Summary');

%% ================================================================
% END
% ================================================================