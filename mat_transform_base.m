% Joint angle ranges in radians
theta1 = -pi/4:deg2rad(5):pi/2;  % [-pi/4, pi/2]
theta2 = -pi/2:deg2rad(5):pi/2;  % [-pi/2, pi/2]

% Initialize workspace points
y_coords = [];
z_coords = [];

% Loop through combinations of theta1 and theta2
for i = 1:length(theta1)
    for j = 1:length(theta2)
        th1 = theta1(i);
        th2 = theta2(j);
        
        % Transformation matrix
        x = [0 0 1 -1;
             sin(th1 + th2) cos(th1 + th2) 0 (sin(th1 + th2) + cos(th1));
             -cos(th1 + th2) sin(th1 + th2) 0 (sin(th1) - cos(th1 + th2) + 1);
             0 0 0 1];
        
        % Extract tool origin (column 4 of the matrix)
        tool_origin = x(1:3, 4);
        
        % Store y and z coordinates
        y_coords = [y_coords; tool_origin(2)];
        z_coords = [z_coords; tool_origin(3)];
    end
end

% Plot workspace
figure;
hold on;
plot(y_coords(1:2:end), z_coords(1:2:end), 'o', 'DisplayName', 'Set 1');
plot(y_coords(2:2:end), z_coords(2:2:end), 'x', 'DisplayName', 'Set 2');
axis equal;
xlabel('y-coordinate');
ylabel('z-coordinate');
title('Workspace in y-z Plane');
legend show;
grid on;