
clear; clc; close all;


% theta = joint angle
% d =  dist along previous z axis in mm
% a = dist along current x axis in mm
% alpha = twist angle about current x axis in mm
% sigma = joint type. 0 is revolute
% offset = const angle added to joint variable

% joint angles are measured in radians

%DH table
% [min angle, max angle]

L(1) = Link([0 330  50 -pi/2 0 0]);
L(2) = Link([0   0 330   0   0 -pi/2]);
L(3) = Link([0   0  35 -pi/2 0 0]);
L(4) = Link([0 335   0  pi/2 0 0]);
L(5) = Link([0   0   0 -pi/2 0 0]);
L(6) = Link([0  80   0   0   0 0]);

robot = SerialLink(L, 'name', 'FANUC LR Mate 200iD');

%Angle limits

AngleLimit = deg2rad([-170   170;-100   145;-205    70;-190   190;-125   125;-360   360 ]);
robot.tool = transl(0, 0, 100);   % toolbox pen, 100 mm along flange z
robot.qlim = AngleLimit;  % joint limits

JointID = {'J1','J2','J3','J4','J5','J6'};

%Generates a usable circular path for the arm

SphereCenter = [450; 50; 150];    % center
Radius = 100;                 % radius
N = 100;

IntermediaryAngles = linspace(0, 2*pi, N+1);
IntermediaryAngles(end) = [];

% Point the center of the path toward the provided coordinates
PathNormalVector = [-0.3; -1; 1];  
PathNormalVector = PathNormalVector / norm(PathNormalVector);  % normalized vector

ReferanceVector = [0; 0; 1];  % chosen reference vector to create the axes within the circle plane
if abs(dot(ReferanceVector, PathNormalVector)) > 0.95
    ReferanceVector = [0; 1; 0];
end

% create first unit vector
UnitVector1 = cross(PathNormalVector, ReferanceVector);
UnitVector1 = UnitVector1 / norm(UnitVector1);

% create second unit vector
UnitVector2 = cross(PathNormalVector, UnitVector1);
UnitVector2 = UnitVector2 / norm(UnitVector2);

% locate circle on the sphere, 30 degrees from normal direction
% circle centre distance from sphere center
PathToCenterDistance = Radius * cosd(30);
RadiusOfPath = sqrt(Radius^2 - PathToCenterDistance^2);  % calculate circle radius with pythagorean

% Each column is one x y z position - Pathcoords
% Each slice of Pathtransformation is 4x4 matrix
PathCoords = zeros(3, N);
PathTransforms = zeros(4, 4, N);

% Target tool poses
for k = 1:N

    % calculate point on circle
    % sphere centre + offset to circle centre + circle displacement
    PathCoords(:,k) = SphereCenter + PathToCenterDistance*PathNormalVector + RadiusOfPath*(UnitVector1*cos(IntermediaryAngles(k)) + UnitVector2*sin(IntermediaryAngles(k)));
    
    % point tool inward toward the sphere centre - current z axis
    zToolVector = (SphereCenter - PathCoords(:,k)) / Radius;

    % reference direction
    ref_tool = [1; 0; 0];
    if abs(dot(ref_tool, zToolVector)) > 0.95
        ref_tool = [0; 1; 0];
    end

    % creates and normalizes local x axis for the tool
    xToolVector = cross(ref_tool, zToolVector);
    xToolVector = xToolVector / norm(xToolVector);

    % creates tools local y axis for right handed frame
    yToolVector = cross(zToolVector, xToolVector);
    yToolVector = yToolVector / norm(yToolVector);
    
    % the tools xyz axes in base frame
    RotationMatrix = [xToolVector, yToolVector, zToolVector];
    PathTransforms(:,:,k) = [RotationMatrix, PathCoords(:,k); 0 0 0 1]; % desired o
end

% start config

% Start sim in center position of joints
% start at midpoint of all joints angle limits
StartingAngles = deg2rad([0 22.5 -67.5 0 0 0]);

% scale unit sphere and translate to centre
[Xs, Ys, Zs] = sphere(40);
Xs = Radius*Xs + SphereCenter(1);
Ys = Radius*Ys + SphereCenter(2);
Zs = Radius*Zs + SphereCenter(3);

% PLOT
%800 so robot fits inside graphing area
SizeOfGraph = 800;
xmin = min([0; PathCoords(1,:)']) - SizeOfGraph;
xmax = max([0; PathCoords(1,:)']) + SizeOfGraph;
ymin = min([0; PathCoords(2,:)']) - SizeOfGraph;
ymax = max([0; PathCoords(2,:)']) + SizeOfGraph;
zmin = min([0; PathCoords(3,:)']);
zmax = max([0; PathCoords(3,:)']) + SizeOfGraph;

% visualization
figure(1);

% draw robot in starting config
robot.plot(StartingAngles, ...
    'workspace', [xmin xmax ymin ymax zmin zmax], ...
    'nowrist', 'noname', 'scale', 1.0);
hold on;

% draws sphere
surf(Xs, Ys, Zs, ...
    'FaceColor', 'cyan', ...
    'FaceAlpha', 0.25, ...
    'EdgeColor', [0.4 0.6 0.7]);

% draws circular path red
plot3(PathCoords(1,:), PathCoords(2,:), PathCoords(3,:), ...
    'r-', 'LineWidth', 2);

plot3(SphereCenter(1), SphereCenter(2), SphereCenter(3), ...
    'ko', 'MarkerFaceColor', 'k');

% equal axis scaled so sphere keeps correct shape
axis equal;
axis([xmin xmax ymin ymax zmin zmax]);
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('FANUC LR Mate 200id');
view(135, 25);
drawnow;

%Inverse Kinematics section

% sets one row of six joint angles for all every path point
ArmTrajectory = zeros(N, 6);

% initialize solver start guess
TrajectorySeed = StartingAngles;

% tracks how many consecutive path points that are solved
LastValidTrajectory = 0;

% Try random starting guesses for the first point, keep the best one
rng(1);   % same random guesses every run
BestError = inf;

% Tries 20 starting guesses
for s = 1:20

    % randomly chooses starting angle
    q0 = AngleLimit(:,1)' + rand(1,6) .* (AngleLimit(:,2) - AngleLimit(:,1))';
    [qTry, errTry] = robot.ikcon(PathTransforms(:,:,1), q0);
    if errTry < BestError
        BestError = errTry;
        TrajectorySeed = qTry;
    end
end

% Solve current target tool pose
for k = 1:N

    % solve for first target pose
    try
        TrajectorySolution = robot.ikcon(PathTransforms(:,:,k), TrajectorySeed);

        % keeps solution with smallest objective value
        if isempty(TrajectorySolution) || any(~isfinite(TrajectorySolution(:)))
            warning('IK failed at point %d.', k);
            break;
        end

        TrajectorySolution = TrajectorySolution(end,:);

        % Check next position
        TrajectoryCheck = robot.fkine(TrajectorySolution);
        if isa(TrajectoryCheck, 'SE3')
            TrajectoryCheck = TrajectoryCheck.T;
        end

        % error handler for NaN and inf values
        if norm(TrajectoryCheck(1:3,4) - PathCoords(:,k)) > 2
            warning('Position error too large at point %d.', k);
            break;
        end

        % Check joint limits. This sim is designed to warn you when you
        % violate the joint limits. It does not however stop the sim so you
        % can still figure out whats going on
        for j = 1:6
            if TrajectorySolution(j) < AngleLimit(j,1) || TrajectorySolution(j) > AngleLimit(j,2)
                warning(['Checkpoint %d: Joint: %s = %.2f deg allowed range [%.2f, %.2f]'], k, JointID{j}, rad2deg(TrajectorySolution(j)),rad2deg(AngleLimit(j,1)), rad2deg(AngleLimit(j,2)));
            end
        end

        % Save IK solution
        ArmTrajectory(k,:) = TrajectorySolution;
        TrajectorySeed = TrajectorySolution;
        LastValidTrajectory = k;

    catch Error
        warning('IK failed at %d: %s', k, Error.message);
        break;
    end
end


% Validate the calculated circle trajectory before animation.
if LastValidTrajectory ~= N
    error('Not all path points solved.');
end

% Measure position and orientation errors
PositionError = zeros(N,1);
OrientationError = zeros(N,1);

% Solve path points
for k = 1:N
    F = robot.fkine(ArmTrajectory(k,:));

    if isa(F, 'SE3')
        F = F.T;
    end

    % distance between actual and target tool tip position in mm
    PositionError(k) = norm(F(1:3,4) - PathCoords(:,k));

    % Converts relative rotation matrix into rotation angle
    Rerror = PathTransforms(1:3,1:3,k)' * F(1:3,1:3);
    cosine = (trace(Rerror)-1)/2;
    OrientationError(k) = acosd(max(-1, min(1, cosine)));
end

% Check joint steps and limits

% Calculate joint changes between points
JointChanges = rad2deg(diff([ArmTrajectory; ArmTrajectory(1,:)]));

% Find largest absolute change for joint for different steps
MaxJointStep = max(abs(JointChanges(:)));

% Checks if all saved configurations are witin angle limits
WithinLimits = all(all( ...
    ArmTrajectory >= AngleLimit(:,1)' - 1e-6 & ...
    ArmTrajectory <= AngleLimit(:,2)' + 1e-6));

% Prints trajectory
% Prints Orientation error, max joint step, and WithinLimits
fprintf('Max position error: %.4f mm\n', max(PositionError));
fprintf('Max orientation error: %.4f deg\n', max(OrientationError));
fprintf('Max joint step: %.3f deg\n', MaxJointStep);
fprintf('Within limits: %d\n', WithinLimits);
%Run sim

if LastValidTrajectory == N

    % Smooth move from the start pose to the first point on the circle
    ApproachPath = jtraj(StartingAngles, ArmTrajectory(1,:), 50);
    for k = 1:size(ApproachPath, 1)  % % Animation for approach move
        robot.animate(ApproachPath(k,:));
        drawnow;
        pause(0.03);
    end

    % Repeat the trajectory
    while true
        for k = 1:N
            robot.animate(ArmTrajectory(k,:));
            drawnow;
            pause(0.03);
        end
    end

elseif LastValidTrajectory > 0

    for k = 1:LastValidTrajectory
        robot.animate(ArmTrajectory(k,:));
        drawnow;
        pause(0.03);
    end

else

end
