x_values= [100, 105, 103, 102, 99];
y_values= [53, 57, 54, 55, 52];

desired_x= [100, 100, 100, 100, 100];
desired_y = [56, 56, 56, 56, 56];

% Plot points
figure;
hold on;

% Plot points th1 and th2 in the same color
plot(x_values, 'ro', 'MarkerSize', 10, 'DisplayName', 'x values'); % red color
plot(y_values, 'ro', 'MarkerSize', 10, 'DisplayName', 'y values');

% Plot points th3 and th4 in different colors
plot(desired_x, 'bo', 'MarkerSize', 10, 'DisplayName', 'desired x'); % blue color
plot(desired_y, 'go', 'MarkerSize', 10, 'DisplayName', 'desired y'); % green color


% Set axis limits for better visualization
axis([0 8 0 10]);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
title('errors');

legend show;
hold off;

