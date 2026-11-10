function [scnro, egoVehicle, pathPose] = run_apf()
% Dynamic Artificial Potential Field Path Planning with Moving Obstacles

%% Load scenario
[scnro, egoVehicle] = driving_scenario();

%% Read obstacles
actors = scnro.Actors;
obstacles = actors([actors.ActorID] ~= egoVehicle.ActorID);

fprintf('Loaded %d obstacle vehicles.\n', numel(obstacles));

%% Give obstacles a driving behavior
for i = 1:numel(obstacles)
    obs = obstacles(i);
    startPos = obs.Position;
    
    % Die Autos fahren 25 Meter vor und stoppen dann
    obsWaypoints = [startPos; 
                    startPos(1) + 25, startPos(2), startPos(3)];
                
    obsSpeed = 3; % Fahrgeschwindigkeit in m/s
    smoothTrajectory(obs, obsWaypoints, obsSpeed);
end

%% Road boundaries
boundaries = roadBoundaries(scnro);
roadPoints = vertcat(boundaries{:});

roadXLim = [min(roadPoints(:,1)), max(roadPoints(:,1))];
roadYLim = [min(roadPoints(:,2)), max(roadPoints(:,2))];

%% Start and Goal
startPose = [egoVehicle.Position(1:2), egoVehicle.Yaw];

if abs(startPose(1)-roadXLim(1)) < abs(startPose(1)-roadXLim(2))
    goalX = roadXLim(2);
    goalYaw = 0;
else
    goalX = roadXLim(1);
    goalYaw = 180;
end

goalPose = [goalX, mean(roadYLim), goalYaw];

fprintf('Start: %.2f, %.2f\n', startPose(1:2));
fprintf('Goal: %.2f, %.2f\n', goalPose(1:2));

%% Simulation Setup
scnro.SampleTime = 0.05;
desiredSpeed = 8;

%% Visualization Setup mit Legende
figure('Name','Dynamic APF Path Planning');
plot(scnro);
hold on;

hStart = plot(startPose(1), startPose(2), 'bo', 'MarkerSize',9,'MarkerFaceColor','b');
hGoal = plot(goalPose(1), goalPose(2), 'g*', 'MarkerSize',12,'LineWidth',2);
hPath = plot(NaN, NaN, 'b-', 'LineWidth', 2);

title('Dynamic Artificial Potential Field Path Planning');
xlabel('X [m]');
ylabel('Y [m]');
legend([hPath, hStart, hGoal], {'APF Path', 'Start', 'Goal'}, 'Location', 'best');
grid on;
axis equal;

%% DYNAMISCHE ECHTZEIT-SIMULATION (Reaktiver APF-Ansatz)
p = startPose(1:2);
goal = goalPose(1:2);
goalTol = 0.5;
stepSize = 0.4;

pathHistory = p;
dirTotalPrev = [1, 0];

while advance(scnro)
    
    if norm(goal - p) < goalTol
        disp('Ziel erreicht!');
        break;
    end
    
    p = egoVehicle.Position(1:2);
    
    Fatt = 1.0 * (goal - p);
    Frep = [0 0];
    
    influenceDistance = 15.0; 
    activeObstaclesCount = 0; 
    
    for i = 1:numel(obstacles)
        obs = obstacles(i);
        
        % Ignoriere Autos hinter dem Ego-Vehicle
        if obs.Position(1) < p(1) - 2.0
            continue; 
        end
        
        L = obs.Length;
        W = obs.Width;
        yaw = deg2rad(obs.Yaw);

        forward = [cos(yaw), sin(yaw)];
        
        % -> HIER GEÄNDERT: Der Offset wird noch weiter nach hinten verschoben (-3.0 statt -1.0),
        % damit das Ego-Vehicle das Hindernis (insb. das Heck) noch früher erkennt.
        centerOffset = (L/2 - obs.RearOverhang) - 3.0; 
        rectCenter = obs.Position(1:2) + centerOffset*forward;

        R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
        relativePos = (R'*(p-rectCenter)')';

        halfL = L/2 + 0.8; 
        halfW = W/2 + 0.5; 

        closest = [min(max(relativePos(1),-halfL),halfL), ...
                   min(max(relativePos(2),-halfW),halfW)];

        dVecLocal = relativePos - closest;
        dist = norm(dVecLocal);

        if dist < influenceDistance
            if dist > 1e-6
                dirLocal = dVecLocal/dist;
            else
                dirLocal = [1 0];
            end
            direction = (R*dirLocal')';
            dist = max(dist, 0.05);
            
            Frep = Frep + 25.0 * (1/dist - 1/influenceDistance) * (1/dist^2) * direction;
            activeObstaclesCount = activeObstaclesCount + 1;
        end
    end
    
    % Abstoßung von den Straßenrändern
    Fwall = [0 0];
    wallInfluence = 2.0;
    distLower = p(2) - roadYLim(1);
    distUpper = roadYLim(2) - p(2);
    
    if distLower < wallInfluence
        d = max(distLower, 0.05);
        Fwall(2) = Fwall(2) + 8.0 * (1/d - 1/wallInfluence) / d^2;
    end
    
    if distUpper < wallInfluence
        d = max(distUpper, 0.05);
        Fwall(2) = Fwall(2) - 8.0 * (1/d - 1/wallInfluence) / d^2;
    end
    
    % Zentrierungskraft
    centerY = mean(roadYLim);
    if activeObstaclesCount == 0
        centerGain = 0.15; 
    else
        centerGain = 0.02; 
    end
    Fcenter = [0, -centerGain * (p(2) - centerY)];
    
    Ftotal = Fatt + Frep + Fwall + Fcenter;
    
    if norm(Ftotal) > 1e-8
        dirTotal = Ftotal / norm(Ftotal);
        
        if activeObstaclesCount > 0
            smoothingFactor = 0.8;
        else
            smoothingFactor = 0.4; 
        end
        
        dirTotal = smoothingFactor * dirTotalPrev + (1 - smoothingFactor) * dirTotal;
        dirTotal = dirTotal / norm(dirTotal);
        dirTotalPrev = dirTotal;
        
        pNew = p + stepSize * dirTotal;
        pNew(2) = min(max(pNew(2), roadYLim(1)+0.3), roadYLim(2)-0.3);
        
        yawNew = atan2d(dirTotal(2), dirTotal(1));
        
        egoVehicle.Position = [pNew, 0];
        egoVehicle.Yaw = yawNew;
        
        pathHistory(end+1, :) = pNew;
        set(hPath, 'XData', pathHistory(:,1), 'YData', pathHistory(:,2));
    end
    
    pause(scnro.SampleTime);
end

pathPose = [pathHistory, zeros(size(pathHistory,1),1)];
disp('Simulation finished.');

end