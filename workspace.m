% Communication between Arduino and MATLAB 
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

close all
clear all

% Declare global variables
%   hPlot       plot widget
%   hFig        figure widget
%   c           counter
%   x           x coordinate of points
%   y           y coordinate of points
global hPlot hFig c x y 

% Create GUI
hFig = figure;

% Create plot area
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);

% Set up and initialise variables for real-time plotting
c = 0;
x = [];
y = [];

% Initialise geometry of 2-arm robotic system
r = 78;

% Specify the angle resolution for workspace plotting
resolution = 100; % [1..50]
angle1_range = linspace(0, 360, resolution); % [0..360] degrees
angle2_range = linspace(0, 360, resolution); % [0..360] degrees

% Iterate through the given resolution of both angles
for t1 = angle1_range
    for t2 = angle2_range
        % Compute the homogeneous transformation via forward_kinematics()
        [x_temp, y_temp] = forward_kinematics(t1, t2, r);
        
        % Append the computed points to the arrays
        x = [x, x_temp];
        y = [y, y_temp];
        
        % Increase counter to save information 
        c = c + 1;
    end
end

% Real-time plotting
colors = parula(length(x)); % Use a colormap for different colors
scatter(hPlot, x, y, 20, colors, 'filled'); % Scatter with colors
xlim([-(r+r), r+r]);
ylim([-(r+r), r+r]);
hold on

% Draw a 156 by 156 square centered at the origin
rectangle('Position', [-78, -78, 156, 156], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Set title and axis labels
title('Maximum workspace with a 156 by 156 space fitted');
xlabel('X (mm)');
ylabel('Y (mm)');

% Close GUI
% delete(hFig);