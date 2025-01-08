x_d = [30;40];
th1 = 120;
th2 = 30;
r=78;
[new_th1,new_th2] = ik_differential(r,th1,th2, x_d)

% Plot points
figure;
hold on;

% Plot points th1 and th2 in the same color
plot(th1, 'ro', 'MarkerSize', 10, 'DisplayName', 'th1'); % red color
plot(th2, 'ro', 'MarkerSize', 10, 'DisplayName', 'th2');

% Plot points th3 and th4 in different colors
plot(new_th1, 'bo', 'MarkerSize', 10, 'DisplayName', 'new_th1'); % blue color
plot(new_th2, 'go', 'MarkerSize', 10, 'DisplayName', 'new_th2'); % green color

% Draw lines between specified points
plot([th1, new_th1], 'k-', 'LineWidth', 1.5); % line between th1 and th3
plot([th2, new_th2], 'k-', 'LineWidth', 1.5); % line between th2 and th4

% Set axis limits for better visualization
axis([0 8 0 10]);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
title('Plot of Points and Lines');

legend show;
hold off;