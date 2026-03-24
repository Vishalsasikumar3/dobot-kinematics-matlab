% ===============================================================
% WORKSPACE_ANALYSIS_SPHERICAL_COLORED.M
% ---------------------------------------------------------------
% Workspace analysis using spherical-joint sampling for a robot
% defined by a URDF. Workspace points are color-coded by distance
% from the robot base.
%
% Date: 2025-11-22
% ===============================================================

clc; clear; close all;

%% --- Inputs ---
urdfFile = "Dobot_magLite.urdf";     % URDF file
eeName = "link5";                     % End-effector link
samplesTheta = 20;                     % Samples along theta (0 to pi)
samplesPhi = 40;                       % Samples along phi (0 to 2pi)

%% --- Load robot ---
robot = importrobot(urdfFile, "DataFormat", "row");
robot.Gravity = [0 0 -9.81];

if isempty(eeName)
    eeName = robot.BodyNames{end};
end

%% --- Extract revolute joints ---
jointIndices = [];
jointLimits = [];
for i = 1:numel(robot.homeConfiguration)
    jointObj = robot.Bodies{i}.Joint;
    if strcmp(jointObj.Type, 'revolute')
        jointIndices(end+1) = i; %#ok<SAGROW>
        if ~isempty(jointObj.PositionLimits)
            jointLimits(end+1,:) = jointObj.PositionLimits; %#ok<SAGROW>
        else
            jointLimits(end+1,:) = [-pi, pi]; %#ok<SAGROW>
        end
    end
end
numJoints = numel(jointIndices);

%% --- Spherical Sampling ---
fprintf('Performing spherical joint sweep...\n');
positions = [];
distances = [];

for th = linspace(0, pi, samplesTheta)
    for ph = linspace(0, 2*pi, samplesPhi)
        q = zeros(1,numJoints);
        for j = 1:numJoints
            % Map spherical angles to joint range (scaled linearly)
            q(j) = jointLimits(j,1) + (jointLimits(j,2) - jointLimits(j,1)) * mod(th + j*ph,1);
        end
        
        % Compute forward kinematics
        T = getTransform(robot, q, eeName);
        pos = T(1:3,4)';
        positions(end+1,:) = pos; %#ok<SAGROW>
        
        % Distance from base
        distances(end+1,1) = norm(pos); %#ok<SAGROW>
    end
end
fprintf('Spherical sweep complete. Points computed: %d\n', size(positions,1));

%% --- Plot Workspace (Color-coded by Distance) ---
figure('Color','w');
scatter3(positions(:,1), positions(:,2), positions(:,3), 12, distances, 'filled');
colormap turbo; colorbar;
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Workspace via Spherical Joint Sweep (Distance Colored)');
view(120,30);
camlight;

%% --- Optional: Save Data ---
% save('workspace_spherical_colored.mat','positions','distances','jointLimits');

fprintf('Workspace analysis finished.\n');

