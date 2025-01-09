%   Manipulability
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024

close all
clear all
clc

x_d = [1; 1.5]


% Control parameters
damping_factor = 0.01;   % Damping factor for DLS
step_limit = 0.1;        % Maximum step size for target clamping
gain = 0.3;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 1000;   % Maximum number of iterations
dt = 0.01;               % Time step for plotting manipulability
ellipsoid_scale = 0.25;   % Scaling factor for ellipsoids

% Initialize array to store manipulability values
manipulability_values = zeros(max_iterations, 1);

figure;
hold on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('X');
ylabel('Y');
title('Robot Arm Motion and Manipulability Ellipsoids');

r=78
theta= 60
phi= 30

for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
   
    [x,y] = kforward_kinematics(theta, phi, r)
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) >= step_limit
        error = (error/norm(error))*step_limit;
        % task1
    end
    
    % Compute Jacobian for the current joint angles
    kJ = ikik_jacobian(r, theta, phi)
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability = sqrt(det(kJ*kJ')); %task 2 ;
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    kinv_J_damped = (kJ'*kJ+damping_factor^2*eye(size(kJ,2))) \ kJ'; % task 3;
    kdelta_q_dls = kinv_J_damped * error;
    
    % Jacobian Transpose method as an alternative to inverse Jacobian
    %delta_q_transpose = %task 4;
    
    % Select method: uncomment one of the two lines below
    kdelta_q = kdelta_q_dls;     % Damped Least Squares method
    %delta_q = delta_q_transpose; % Jacobian Transpose method
    
    % Update joint angles using calculated change
    theta = theta + kdelta_q(1)
    phi = phi + kdelta_q(2)
    
    % Plot current robot arm position
    plot([0, r*cosd(theta), r*cosd(theta) + r*cosd(theta + phi)], ...
         [0, r*sind(theta), r*sind(theta) + r*sind(theta + phi)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, V] = svd(kJ) %task 5;  % Singular Value Decomposition of the Jacobian
    omega = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * cosd(omega) * S(1, 1) %.....apply scaling;  
    ellipse_y = ellipsoid_scale * sind(omega) * S(2, 2) %.....apply scaling;  
    ellipse = U * [ellipse_x; ellipse_y]; 
    
    % Plot the ellipsoid around the current end-effector position
    plot(ellipse(1, :) + x, ellipse(2, :) + y, 'm');
    
    pause(0.01);  % Pause for visualization
    
    % Check if the end-effector is within the desired tolerance
    if norm(error) < tolerance
        disp('Desired position reached!');
        break;
    end
end

% Plot manipulability over the trajectory
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');
