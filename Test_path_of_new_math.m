
clear; clc; close all;

%DH table

L(1) = Link([0 330  50 -pi/2 0 0]);
L(2) = Link([0   0 330   0   0 -pi/2]);
L(3) = Link([0   0  35 -pi/2 0 0]);
L(4) = Link([0 335   0  pi/2 0 0]);
L(5) = Link([0   0   0 -pi/2 0 0]);
L(6) = Link([0  80   0   0   0 0]);

robot = SerialLink(L, 'name', 'Arm');

% Add tool length to the end of J6
ToolLength = 10;
robot.tool = transl(0, 0, ToolLength);

%Angle limits

AngleLimit = deg2rad([-170   170;-100   145;-205    70;-190   190;-125   125;-360   360 ]);

JointID = {'J1','J2','J3','J4','J5','J6'};



% Sphere parameters
SphereCenter = [420; 420; 500];
Radius = 70;

PathCenter = [0; 0; 400];

%CircleAxis = PathCenter - SphereCenter

CircleAxis = [-0.6973; -0.6973; -0.1660]
%CircleAxis = CircleAxis / norm(CircleAxis)



Beta = deg2rad(72.54);
N = 100;                        % Number of path points

% Normalize the circle axis
CircleAxis = CircleAxis / norm(CircleAxis);

% Construct the two unit vectors defining the circle plane
ReferenceVector = [0; 0; 1];

if abs(dot(CircleAxis, ReferenceVector)) > 0.95
    ReferenceVector = [0; 1; 0];
end


UnitVector1 = cross(ReferenceVector, CircleAxis);
UnitVector1 = UnitVector1 / norm(UnitVector1);

UnitVector2 = cross(CircleAxis, UnitVector1);
UnitVector2 = UnitVector2 / norm(UnitVector2);

% Sample the circle uniformly
IntermediaryAngles = (0:N-1) * (2*pi/N);

% Calculate circle center and radius from the math
CircleCenter = SphereCenter + Radius*cos(Beta)*CircleAxis;
RadiusOfPath = Radius*sin(Beta);

% Allocate path arrays
PathCoords = zeros(3, N);
PathTransforms = zeros(4, 4, N);

for k = 1:N

    theta = IntermediaryAngles(k);

        % Unit surface-normal vector
    SurfaceNormal = cos(Beta)*CircleAxis ...
        + sin(Beta)*(UnitVector1*cos(theta) ...
        + UnitVector2*sin(theta));

    % Desired PEN TIP position on the sphere
    PathCoords(:,k) = SphereCenter + Radius*SurfaceNormal;

    % Match the original working tool orientation
zToolVector = (SphereCenter - PathCoords(:,k)) / Radius;

ref_tool = [1; 0; 0];

if abs(dot(ref_tool, zToolVector)) > 0.95
    ref_tool = [0; 1; 0];
end

xToolVector = cross(ref_tool, zToolVector);
xToolVector = xToolVector / norm(xToolVector);

yToolVector = cross(zToolVector, xToolVector);
yToolVector = yToolVector / norm(yToolVector);

RotationMatrix = [xToolVector, yToolVector, zToolVector];

PathTransforms(:,:,k) = ...
    [RotationMatrix, PathCoords(:,k); 0 0 0 1];
end
%Start sim in center position of joints

StartingAngles = deg2rad([0 22.5 -67.5 0 0 0]);

[Xs, Ys, Zs] = sphere(40);
Xs = Radius*Xs + SphereCenter(1);
Ys = Radius*Ys + SphereCenter(2);
Zs = Radius*Zs + SphereCenter(3);

SizeOfGraph = 600;
xmin = min([0; PathCoords(1,:)']) - SizeOfGraph;
xmax = max([0; PathCoords(1,:)']) + SizeOfGraph;
ymin = min([0; PathCoords(2,:)']) - SizeOfGraph;
ymax = max([0; PathCoords(2,:)']) + SizeOfGraph;
zmin = min([0; PathCoords(3,:)']);
zmax = max([0; PathCoords(3,:)']) + SizeOfGraph;

figure(1);
robot.plot(StartingAngles, ...
    'workspace', [xmin xmax ymin ymax zmin zmax]);
hold on;

surf(Xs, Ys, Zs, ...
    'FaceColor', 'cyan', ...
    'FaceAlpha', 0.25, ...
    'EdgeColor', [0.4 0.6 0.7]);

plot3(PathCoords(1,:), PathCoords(2,:), PathCoords(3,:), ...
    'r-', 'LineWidth', 2);

plot3(SphereCenter(1), SphereCenter(2), SphereCenter(3), ...
    'ko', 'MarkerFaceColor', 'k');

axis equal;
axis([xmin xmax ymin ymax zmin zmax]);
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('FANUC LR Mate 200id');
view(135, 25);
drawnow;

%Ik section

ArmTrajectory = zeros(N, 6);
TrajectorySeed = StartingAngles;
LastValidTrajectory = 0;

for k = 1:N

    try
        TrajectorySolution = robot.ikine( ...
    PathTransforms(:,:,k), TrajectorySeed, ...
    'mask', [1 1 1 1 1 1]);

        if isempty(TrajectorySolution) || any(~isfinite(TrajectorySolution(:)))
            warning('IK failed at point %d.', k);
            break;
        end

        TrajectorySolution = TrajectorySolution(end,:);

        % Do not allow large jumps
        TrajectorySolution = TrajectorySeed + atan2(sin(TrajectorySolution - TrajectorySeed), cos(TrajectorySolution - TrajectorySeed));

        % Check next position
        TrajectoryCheck = robot.fkine(TrajectorySolution);
        if isa(TrajectoryCheck, 'SE3')
            TrajectoryCheck = TrajectoryCheck.T;
        end

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

%Run sim

if LastValidTrajectory == N

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
