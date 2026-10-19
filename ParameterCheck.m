clear; clc; close all;
rng('shuffle');
searchRng = rng;
fprintf('Seed: %d\n', searchRng.Seed);  % Searches randomly for first passing parameters and IK guesses

% DH model, length in mm, angles in rads
L(1) = Link([0 330  50 -pi/2 0 0]);
L(2) = Link([0   0 330   0   0 -pi/2]);
L(3) = Link([0   0  35 -pi/2 0 0]);
L(4) = Link([0 335   0  pi/2 0 0]);
L(5) = Link([0   0   0 -pi/2 0 0]);
L(6) = Link([0  80   0   0   0 0]);
% six links for model
robot = SerialLink(L, 'name', 'FANUC LR Mate 200iD');
Lim = deg2rad([
    -170 170
    -100 145
    -205 215
    -190 190
    -125 125
    -360 360
]);

robot.qlim = Lim;   % Joint limits
robot.tool = transl(0, 0, 100); % pen model 100mm

N = 100; % check 100 points
found = false; 

for trial = 1:100

    % Sample candidate sphere centres inside of selected search region
    SphereCenter = [
        25*randi([18 20])   % X: 450, 475, 500
        25*randi([1 3])     % Y: 25, 50, 75
        25*randi([5 7])     % Z: 125, 150, 175
        ];  % Chosen bounds based on 717mm max reach

    Radius = 100; % Sphere radius in mm

    Axis = [randi([-5 -1])/10; -1; 1];
    Beta = 30;  % 29deg on first, so tried to run with even number 30deg

    a = Axis / norm(Axis); % Normalize axis so len = 1

    % Circle coordinate frame

    [~, j] = min(abs(a));  % coordinate direction least aligned with the normal
    E = eye(3);

    % Constructs 2 perpendicular unit vectors in circles plane
    u = cross(a, E(:,j));
    u = u / norm(u);
    v = cross(a, u);

    theta = linspace(0, 2*pi, N+1);  % Angles for one complete rev
    theta(end) = [];

    % Circular path on sphere
    % Calculate circle on point on sphere
    % Each col is one XYZ position
    P = SphereCenter + Radius*cosd(Beta)*a + ...
        Radius*sind(Beta)*(u*cos(theta) + v*sin(theta));

    % Allocate one target transformation matrix
    T = zeros(4,4,N);

    % Points tool at z axis inward toward sphere
    for k = 1:N
        z = (SphereCenter - P(:,k)) / Radius;
        x = cross(u, z);
        x = x / norm(x);
        y = cross(z, x);

        T(:,:,k) = [x y z P(:,k); 0 0 0 1];
    end

    % First point inverse kinematics
    best = inf;  % Find starting joint config for the first target pose
    Q1 = []; % Best first point joint cofinguration found
    
    % Random six initial joint angles
    for s = 1:8

        % Joint limits in rads
        q0 = Lim(:,1)' + ...
            rand(1,6).*(Lim(:,2)-Lim(:,1))';

        % Solve first target pose
        [q, err, exitflag] = robot.ikcon(T(:,:,1), q0);

        if exitflag > 0 && all(isfinite(q)) && err < best
            best = err;
            Q1 = q;
        end
    end
    
    % Skip if no correct starting solution
    if isempty(Q1)
        fprintf('Trial %d: no starting solution\n', trial);
        continue
    end

    Q = zeros(N,6); % One row of six joint angles
    Q(1,:) = Q1;  % Store joint config
    solved = true;

    % Remaining path poses
    for k = 2:N
        [q, ~, exitflag] = robot.ikcon(T(:,:,k), Q(k-1,:));

        % Rejects if solver fails or invalid angles
        if exitflag <= 0 || any(~isfinite(q))
            solved = false;
            break
        end

        % Solve remaining path
        Q(k,:) = q;
    end
    % Allocate array for position and full orientation error
    if ~solved
        fprintf('Trial %d: path solver failed\n', trial);
        continue
    end

    % Position and tool axis errors
    % For a rotation matrix, cos(angle) = (trace(R)-1)/2.
    posErr = zeros(N,1); % Tool tip position error
    angleErr = zeros(N,1);  % Tool z axis alignment error in deg

    for k = 1:N
        
        % Calculate tool tip  pose  using forward kinematics
        F = robot.fkine(Q(k,:));

        if isa(F, 'SE3')
            F = F.T;
        end

        posErr(k) = norm(F(1:3,4) - P(:,k));

        % angle between actual tool z-axis and required z-axis
        angleErr(k) = acosd(min(1, dot(F(1:3,3), T(1:3,3,k))));
    end

    % Joint continuity and limits

    changes = diff([Q; Q(1,:)], 1, 1);
    maxStep = max(max(abs(rad2deg(changes))));

    % Checks that every joint angle is within limit
    withinLimits = all(all( ...
        Q >= Lim(:,1)' - 1e-6 & ...
        Q <= Lim(:,2)' + 1e-6));

    % report max errors and joint step
    fprintf('Trial %d: %.3f mm, %.3f deg, %.1f deg step\n', ...
        trial, max(posErr), max(angleErr), maxStep);

    % criterias
    % presents first param that passes thresholds

    % max position error < 1 mm
    % max tool-axis alignment error < 0.5 degrees
    % max joint step < 10 degrees
    % all sampled joint configurations within limits
    if max(posErr) < 1 && max(angleErr) < 0.5 && ...
            maxStep < 10 && withinLimits
        found = true;
        break
    end
end

% Search result
if ~found
    error('No passing params.');  % No passing params retry with new seed
end

% Displays first passing parameters
fprintf('\nPASSING PARAMS\n');
fprintf('SphereCenter = %s mm\n', mat2str(SphereCenter, 5));
fprintf('Radius = %.3f mm\n', Radius);
fprintf('Axis = %s\n', mat2str(Axis, 5));
fprintf('Beta = %.3f deg\n', Beta);
fprintf('Max position error = %.4f mm\n', max(posErr));
fprintf('Max orientation err = %.4f deg\n', max(angleErr));
fprintf('Max joint step = %.3f deg\n', maxStep);

save('sphere_search_result.mat', ...
    'SphereCenter', 'Radius', 'Axis', 'Beta', ...
    'Q', 'P', 'posErr', 'angleErr', 'maxStep', 'Lim');

% Plots the passing parameters
robot.plot(Q(1,:));
hold on;

% Creates sphere
[X,Y,Z] = sphere(30);  % sphere

% Scale and translate
surf(SphereCenter(1)+Radius*X, ...
     SphereCenter(2)+Radius*Y, ...
     SphereCenter(3)+Radius*Z, ...
     'FaceAlpha', 0.2, 'EdgeColor', 'none');

% Draw tool tip path
plot3(P(1,:), P(2,:), P(3,:), 'r-', 'LineWidth', 2);

% axis in mm
axis equal;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');