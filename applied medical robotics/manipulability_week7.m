%   Manipulability
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024

close all
clear all
clc


% Desired position
x_d = [1.5; 1.0]; % Desired x and y coordinates of the end-effector

% Control parameters
damping_factor = 0.01;   % Damping factor for DLS
step_limit = 0.1;        % Maximum step size for target clamping
gain = 0.3;              % Gain for Jacobian transpose method
tolerance = 1e-3;        % Position error tolerance
max_iterations = 10;   % Maximum number of iterations
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

r=1
th1= -0.2
th2= -1

for i = 1:max_iterations
    % Calculate forward kinematics to get the current end-effector position
   
    [x,y] = forward_kinematics(th1, th2, r)
    current_position = [x; y];
    
    % Calculate position error between desired and current end-effector position
    error = x_d - current_position;
    
    % Target clamping: Limit the step size to avoid large jumps
    if norm(error) >= step_limit
        error = (error/norm(error))*step_limit;
        % task1
    end
    
    % Compute Jacobian for the current joint angles
    J = ik_jacobian(r,th1,th2)
    
    % Calculate manipulability as the square root of the determinant of (J*J')
    manipulability = sqrt(det(J*J')); %task 2 ;
    manipulability_values(i) = manipulability;
    
    % Damped Least Squares (DLS) method for stable inverse calculation
    inv_J_damped = (J' * J + damping_factor^2 * eye(size(J,2))) \ J'; % task 3;
    delta_q_dls = inv_J_damped * error;
    
    % Jacobian Transpose method as an alternative to inverse Jacobian
    delta_q_transpose = gain * J' * error;
    
    % Select method: uncomment one of the two lines below
    %delta_q = delta_q_dls;     % Damped Least Squares method
    delta_q = delta_q_transpose; % Jacobian Transpose method
    
    % Update joint angles using calculated change
    th1 = th1 + delta_q(1)
    th2 = th2 + delta_q(2)
    
    % Plot current robot arm position
    plot([0, r*cos(th1), r*cos(th1) + r*cos(th1 + th2)], ...
         [0, r*sin(th1), r*sin(th1) + r*sin(th1 + th2)], '-o');
    
    % Plot the manipulability ellipsoid at the current configuration
    [U, S, V] = svd(J) %task 5;  % Singular Value Decomposition of the Jacobian
    theta = linspace(0, 2*pi, 100);
    ellipse_x = ellipsoid_scale * cos(theta) * S(1, 1) %.....apply scaling;  
    ellipse_y = ellipsoid_scale * sin(theta) * S(2, 2) %.....apply scaling;  
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

% Plot manipulability over the trajectory, PLOTTING FOR SERIAL
figure;
plot(manipulability_values(1:i));
xlabel('Iteration');
ylabel('Manipulability');
title('Manipulability across the trajectory');

%Need to make sure it all uses parallel