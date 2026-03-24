%% simulate_dobot_fk_ik.m
% Verify FK/IK against a URDF robot in MATLAB and animate the motion.
% Requirements:
%   - Robotics System Toolbox
%   - fk_dobot.m, ik_dobot.m in the MATLAB path
%
% Conventions:
%   - Angles: radians
%   - Positions: millimeters (mm) for FK/IK; meters (m) for rigidBodyTree

clear; clc; close all;

%% 1) Load robot from URDF (meters)
urdfPath = 'C:\Users\aniru\Downloads\assignment_2_ras\modelrobot.urdf';
if ~isfile(urdfPath)
    error('URDF not found at: %s', urdfPath);
end
robot = importrobot(urdfPath,'DataFormat','row');
if isprop(robot,'Gravity'); robot.Gravity = [0 0 -9.81]; end
showdetails(robot)

% Choose an end-effector link
eeName = char(chooseEndEffector(robot));
fprintf('Using end-effector link: %s\n', eeName);

% Detect whether URDF actually has nonzero fixed transforms (true geometry)
haveOffsets = hasNonzeroOffsets(robot);

% Show the robot (for visuals only)
figure('Name','URDF Robot'); 
show(robot); title('Dobot Magician Lite (URDF)'); axis equal;

% Pull joint limits if present (for first 3 joints only)
limits = getJointLimits(robot, 3);

%% 2) Parameters for FK/IK (millimeters)
pars = struct('a1',53.5,'a2',150,'a3',150,'z_offset',53.5,'r_const',90);

%% 3) FK verification with your joint sets (deg)
% NOTE: Robot has 3 actuated joints; the 4th value in each row is ignored by the arm.
qTests_deg = [ ...
    0    0   0   0;
   90   20  30   0;   % j1, j2, j3, j4
   -2   25  23  45;
   31   52  42   0;
   27   52  24   0];
qTests = deg2rad(qTests_deg);

fprintf('\n=== FK verification (your FK vs rigidBodyTree) ===\n');
for k = 1:size(qTests,1)
    q = qTests(k,1:3);                 % first 3 for FK/IK
    % Your FK (mm)
    Pmm = fk_dobot(q(1),q(2),q(3),pars);
    % RBT FK (m) -> mm (will be [0 0 0] if URDF lacks offsets)
    T    = getTransform(robot, padJoints(qTests(k,:), robot), eeName);
    Pmm_rbt = tform2trvec(T).'*1000; 
    err = Pmm_rbt - Pmm(:);
    fprintf('q = [%6.2f %6.2f %6.2f] deg  |  Δ = [%7.3f %7.3f %7.3f] mm\n', ...
            rad2deg(q), err(1), err(2), err(3));
end

%% 4) Animate FK SLOWLY (same style as IK) across your FK poses
timePerSeg = 0.7;                                % shared timing with IK
nFK = size(qTests,1);
tfFK = timePerSeg*(nFK-1);
tptsFK = linspace(0,tfFK,nFK);
tvecFK = linspace(0,tfFK,max(2,round(30*tfFK)));

% Use only the first 3 actuated joints for FK animation
qFK = qTests(:,1:3);
qFKTraj = makeSmoothTraj(qFK, tptsFK, tvecFK);   % [samples x 3]

figure('Name','FK animation (smooth)'); ax = axes;
show(robot, padJoints(qFK(1,:),robot), 'Parent',ax,'Frames','off');
view(135,20); axis(ax,'equal'); axis(ax,'vis3d'); grid(ax,'on'); hold(ax,'on');

% Pre-compute axis limits from FK trajectory (FK → meters) so it's never cut off
setAxesFromFK(ax, qFKTraj, pars);

txtFK = text(ax, 0.02, 0.98, "", 'Units','normalized', ...
    'HorizontalAlignment','left','VerticalAlignment','top', ...
    'FontSize',10,'BackgroundColor','w','Margin',3);

eeTraceFK = animatedline('Parent',ax,'LineWidth',1.5);

for k = 1:size(qFKTraj,1)
    qk = qFKTraj(k,:);
    show(robot, padJoints(qk,robot), 'PreservePlot',false,'Parent',ax,'Frames','off');

    % Position from YOUR FK (mm), always positive in the overlay
    Pmm = fk_dobot(qk(1), qk(2), qk(3), pars);
    set(txtFK, 'String', composeOverlay(abs(Pmm(:)), rad2deg(qk)));

    % Trace in meters (same axes as show)
    Pm = Pmm(:)/1000;
    addpoints(eeTraceFK, Pm(1), Pm(2), Pm(3));

    title(ax, sprintf('FK follow: t = %.2f s', tvecFK(k)));
    drawnow;
end

%% 5) IK verification using your target positions (mm)
% Format given: [x y z r]; r is ignored (3-DOF arm)
targets_xyzr = [ ...
    240     0   150 ;
      0  272.5  68.54;
    300    50   100 ;
    280  -195    15];
waypoints_mm = targets_xyzr(:,1:3);

q0 = [0 0 0];    % IK seed (radians)
qList = zeros(size(waypoints_mm,1),3);
fprintf('\n=== IK verification (your IK; millimeter targets) ===\n');
for i = 1:size(waypoints_mm,1)
    tgt = waypoints_mm(i,:);
    [q_sol, ok, info] = ik_dobot(tgt(1),tgt(2),tgt(3),pars,q0); %#ok<ASGLU>

    if ~ok
        if i==1, q_sol = q0; else, q_sol = qList(i-1,:); end
        status = 'FAILED (using previous)';
    else
        status = 'OK';
        % Clamp to limits if defined
        for j=1:3
            lo = limits(j,1); hi = limits(j,2);
            if isfinite(lo) && isfinite(hi) && lo < hi
                q_sol(j) = min(max(q_sol(j),lo),hi);
            end
        end
    end

    % FK back-check from YOUR FK (mm)
    Pmm = fk_dobot(q_sol(1), q_sol(2), q_sol(3), pars);
    err = Pmm(:) - tgt(:);

    % Print joint angles (deg) and position/error; position printed as positive mm
    fprintf('Waypoint %d: %s | q(deg)=[%7.2f %7.2f %7.2f] | pos(mm,+)=[%7.2f %7.2f %7.2f] | |err|=%.2f mm\n', ...
        i, status, rad2deg(q_sol), abs(Pmm(1)), abs(Pmm(2)), abs(Pmm(3)), norm(err));

    qList(i,:) = q_sol;
    q0 = q_sol; % warm start
end

% Smooth joint-space trajectory through the IK configs (same timing as FK)
nPts = size(qList,1);
tf = timePerSeg*(nPts-1);
tpts = linspace(0,tf,nPts);
tvec = linspace(0,tf, max(2,round(30*tf)));
qTraj = makeSmoothTraj(qList, tpts, tvec);       % [samples x 3]

% Animate IK path (with annotation and safe axis limits)
figure('Name','IK animation'); ax2 = axes;
show(robot, padJoints(qList(1,:),robot), 'Parent',ax2,'Frames','off');
view(135,20); axis(ax2,'equal'); axis(ax2,'vis3d'); grid(ax2,'on'); hold(ax2,'on');

% Pre-compute axis from FK of the whole IK path
setAxesFromFK(ax2, qTraj, pars);

txtIK = text(ax2, 0.02, 0.98, "", 'Units','normalized', ...
    'HorizontalAlignment','left','VerticalAlignment','top', ...
    'FontSize',10,'BackgroundColor','w','Margin',3);

eeTraceIK = animatedline('Parent',ax2,'LineWidth',1.5);

for k = 1:size(qTraj,1)
    qk = qTraj(k,:);
    show(robot, padJoints(qk,robot), 'Parent',ax2,'PreservePlot',false,'Frames','off');

    % Position from YOUR FK (mm), always positive in the overlay
    Pmm = fk_dobot(qk(1), qk(2), qk(3), pars);
    set(txtIK, 'String', composeOverlay(abs(Pmm(:)), rad2deg(qk)));

    % Trace in meters
    Pm = Pmm(:)/1000;
    addpoints(eeTraceIK, Pm(1), Pm(2), Pm(3));

    title(ax2, sprintf('IK follow: t = %.2f s', tvec(k)));
    drawnow;
end

%% 6) (Optional) Compare to MATLAB numerical IK — only if URDF has offsets
if haveOffsets
    ik = inverseKinematics('RigidBodyTree',robot);
    weights = [1 1 1 0 0 0];
    qNum = zeros(nPts, numel(homeConfiguration(robot)));
    qSeed = padJoints(qList(1,:),robot);
    for i=1:nPts
        tgt_m = waypoints_mm(i,:)/1000;            % m
        Tgoal = trvec2tform(tgt_m);
        [qSolRow,~] = ik(eeName,Tgoal,weights,qSeed);
        qNum(i,:) = qSolRow;
        qSeed = qSolRow;
    end
    delta = qList - qNum(:,1:3);
    d = rad2deg(atan2(sin(delta), cos(delta)));
    fprintf('\nMean |Δjoint| vs MATLAB IK (first 3 joints): [%.2f %.2f %.2f] deg\n', mean(abs(d),1));
else
    fprintf('\n[Note] Skipping MATLAB numerical IK compare: URDF appears to have zero fixed transforms.\n');
end

%% ---------------------- Local helper functions -------------------------
function limits = getJointLimits(robot, K)
    hc = homeConfiguration(robot);
    nJ = numel(hc);
    K = min(K, nJ);
    limits = nan(K,2);
    jcount = 0;
    for i = 1:numel(robot.Bodies)
        j = robot.Bodies{i}.Joint;
        if ismember(j.Type, {'revolute','prismatic'})
            jcount = jcount + 1;
            limits(jcount,:) = j.PositionLimits;
            if jcount >= K, break; end
        end
    end
    if any(~isfinite(limits),"all")
        limits(~isfinite(limits)) = 0;
    end
end

function qRow = padJoints(qin, robot)
    hc = homeConfiguration(robot);
    nJ = numel(hc);
    qRow = zeros(1,nJ);
    M = min(numel(qin), nJ);
    qRow(1:M) = qin(1:M);
end

function ee = chooseEndEffector(robot)
    names = string(robot.BodyNames);
    prefs = ["tool","ee","end_effector","gripper","tip"];
    hit = prefs(ismember(lower(prefs), lower(names)));
    if ~isempty(hit)
        ee = hit(1);
    else
        ee = names(end);
    end
end

function qTraj = makeSmoothTraj(qWaypoints, tpts, tvec)
% Robust trajectory generator -> [numel(tvec) x nJoints]
% Tries trapveltraj with/without 'TimePoints', then cubicpolytraj, then interp1.
    qWaypointsT = qWaypoints.'; % [nJoints x nWpts]
    nSamp = numel(tvec);
    try
        qTraj = trapveltraj(qWaypointsT, nSamp, 'TimePoints', tpts).';
        return
    catch
    end
    try
        qTraj = cubicpolytraj(qWaypointsT, tpts, tvec).';
        return
    catch
    end
    try
        qTraj = trapveltraj(qWaypointsT, nSamp).';
        return
    catch
    end
    qTraj = interp1(tpts, qWaypoints, tvec, 'pchip');
end

function setAxesFromFK(ax, qTraj, pars)
% Compute safe axis limits from YOUR FK path + nominal reach (meters).
    pts_m = zeros(size(qTraj,1),3);
    for k = 1:size(qTraj,1)
        Pmm = fk_dobot(qTraj(k,1), qTraj(k,2), qTraj(k,3), pars);
        pts_m(k,:) = (Pmm(:)/1000).';
    end
    m = 0.05; % 5 cm margin
    mn = min(pts_m,[],1) - m;
    mx = max(pts_m,[],1) + m;

    % Include nominal reach radius from link lengths (meters)
    R = (abs(pars.a2) + abs(pars.a3) + abs(pars.r_const) + 100)/1000; % +100 mm buffer
    xr = [-R R]; yr = [-R R]; zr = [0 max(R, mx(3))];

    xlim(ax, [min(mn(1), xr(1)) max(mx(1), xr(2))]);
    ylim(ax, [min(mn(2), yr(1)) max(mx(2), yr(2))]);
    zlim(ax, [min(mn(3), zr(1)) max(mx(3), zr(2))]);

    axis(ax,'equal'); axis(ax,'vis3d');
    view(ax, 135, 20); grid(ax,'on');
end

function s = composeOverlay(pmm_pos, qdeg)
% Annotation string. Position displayed as positive mm as requested.
    pmm_pos = pmm_pos(:).'; qdeg = qdeg(:).';
    s = sprintf('Pos (mm, +): [%7.2f  %7.2f  %7.2f]\nq (deg):      [%7.2f  %7.2f  %7.2f]', ...
        pmm_pos(1), pmm_pos(2), pmm_pos(3), qdeg(1), qdeg(2), qdeg(3));
end

function tf = hasNonzeroOffsets(robot)
% Returns true if ANY joint has a non-identity fixed transform.
    tf = false;
    for i = 1:numel(robot.Bodies)
        j = robot.Bodies{i}.Joint;
        JP = j.JointToParentTransform;
        CJ = j.ChildToJointTransform;
        if any(abs(tform2trvec(JP))>1e-12) || any(abs(tform2trvec(CJ))>1e-12) ...
           || any(abs(tform2eul(JP))>1e-12) || any(abs(tform2eul(CJ))>1e-12)
            tf = true; return;
        end
    end
end