% Heart Shape Motion Simulation with Symmetric Four-Linkage Arm
% @author Mirroyal Ismayilov
% @organisation King's College London
% @module Applied Medical Robotics
% @year 2024

close all;
clear;
clc;

% Parameters for robot arm
r1 = 0.78; % Length of first link [metre]
r2 = 0.78; % Length of second link
t1 = -0.2; % Initial angle of joint 1
t2 = -1; % Initial angle of joint 2

% Heart shape parameters
t = linspace(0, 2*pi, 1000); % Parameter t for the heart curve
x_d_points = 16 * sin(t).^3; % x-coordinates of the heart shape
y_d_points = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t); % y-coordinates
scale = 0.1; % Scaling factor to fit the heart within reachable workspace
x_d_points = scale * x_d_points;
y_d_points = scale * y_d_points;

% Control parameters
damping_factor = 0.01; % Damping factor for DLS
step_limit = 0.1; % Maximum step size for target clamping
gain = 0.3; % Gain for Jacobian transpose method
tolerance = 1e-3; % Position error tolerance
max_iterations = length(t); % Maximum number of iterations
dt = 0.01; % Time step for smoother motion
ellipsoid_scale = 0.25; % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

% Arrays to store motion points
trajectory_A = []; % Trajectory of point A
trajectory_D = []; % Trajectory of point D
trajectory_C = []; % Trajectory of point C
trajectory_B = []; % Trajectory of point B

% Figure setup
figure;
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X');
ylabel('Y');
title('Symmetric Four-Linkage Robot Arm Heart Shape Motion');

% Plot the desired heart shape for reference
plot(x_d_points, y_d_points, 'r--', 'LineWidth', 1.5);

% Main simulation loop
for i = 1:max_iterations
    % Update target position for heart shape
    x_d = [x_d_points(i); y_d_points(i)];

    % Forward kinematics to calculate current end-effector position
    T = forward_kinematics(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y];

    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;

    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) > step_limit
        error = step_limit * (error / norm(error));
    end

    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r1, r2, t1, t2);

    % Calculate manipulability
    manipulability = sqrt(det(J * J'));
    manipulability_values(i) = manipulability;

    % Eigenvalues and eigenvectors for manipulability ellipsoid
    [V, D] = eig(J * J'); % Eigen decomposition
    eigenvalues = diag(D); % Extract eigenvalues
    a = ellipsoid_scale * sqrt(eigenvalues(1)); % Semi-major axis length
    b = ellipsoid_scale * sqrt(eigenvalues(2)); % Semi-minor axis length
    theta = atan2(V(2, 1), V(1, 1)); % Orientation of ellipsoid

    % Plot manipulability ellipsoid
    ellipse_x = a * cos(t) * cos(theta) - b * sin(t) * sin(theta);
    ellipse_y = a * cos(t) * sin(theta) + b * sin(t) * cos(theta);
    plot(x + ellipse_x, y + ellipse_y, 'k--'); % Ellipsoid at end-effector

    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J + damping_factor^2 * eye(size(J, 2))) \ J';
    delta_q_dls = inv_J_damped * error;

    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error;

    % Select method: uncomment one of the two lines below
    % delta_q = delta_q_dls; % Damped Least Squares method
    delta_q = delta_q_transpose; % Jacobian Transpose method

    % Update joint angles using calculated change
    t1 = t1 + delta_q(1);
    t2 = t2 + delta_q(2);

    % Calculate positions of points A, D, C
    x_A = 0; y_A = 0; % Point A
    x_D = r1*cos(t1); y_D = r1*sin(t1); % Point D
    x_C = x_D + r2*cos(t1 + t2); y_C = y_D + r2*sin(t1 + t2); % Point C

    % Calculate symmetric point B
    AC_vector = [x_C - x_A; y_C - y_A]; % Vector AC
    D_vector = [x_D - x_A; y_D - y_A]; % Vector AD
    proj_D_on_AC = (dot(D_vector, AC_vector) / norm(AC_vector)^2) * AC_vector; % Projection of D on AC
    B_vector = 2 * proj_D_on_AC - D_vector; % Symmetric vector for B
    x_B = x_A + B_vector(1); % Point B x-coordinate
    y_B = y_A + B_vector(2); % Point B y-coordinate

    % Save trajectory points
    trajectory_A = [trajectory_A; x_A, y_A];
    trajectory_D = [trajectory_D; x_D, y_D];
    trajectory_C = [trajectory_C; x_C, y_C];
    trajectory_B = [trajectory_B; x_B, y_B];

    % Plot original links AD and CD
    plot([x_A, x_D], [y_A, y_D], 'b-o', 'LineWidth', 2); % Link AD
    plot([x_D, x_C], [y_D, y_C], 'g-o', 'LineWidth', 2); % Link CD

    % Plot symmetric links AB and BC
    plot([x_A, x_B], [y_A, y_B], 'r-o', 'LineWidth', 2); % Link AB
    plot([x_B, x_C], [y_B, y_C], 'm-o', 'LineWidth', 2); % Link BC

    % Plot trajectories of points
    plot(trajectory_C(:,1), trajectory_C(:,2), 'g-', 'LineWidth', 1); % Trajectory of C

    pause(dt); % Pause for visualization
end

% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:max_iterations));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the heart shape trajectory');

%% Function Definitions

% Forward kinematics function
function T = forward_kinematics(r1, r2, t1, t2)
    % Compute transformation matrices
    T1 = [cos(t1), -sin(t1), 0, r1*cos(t1);
          sin(t1),  cos(t1), 0, r1*sin(t1);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T2 = [cos(t2), -sin(t2), 0, r2*cos(t2);
          sin(t2),  cos(t2), 0, r2*sin(t2);
          0,        0,       1, 0;
          0,        0,       0, 1];
    T = T1 * T2; % Combined transformation
end

% Jacobian calculation function
function J = ik_jacobian(r1, r2, t1, t2)
    % Compute the Jacobian for a 2-link planar manipulator
    J11 = -r1*sin(t1) - r2*sin(t1 + t2);
    J12 = -r2*sin(t1 + t2);
    J21 =  r1*cos(t1) + r2*cos(t1 + t2);
    J22 =  r2*cos(t1 + t2);
    J = [J11, J12; J21, J22];
end   
