clear
% Communicaiton between Arduino and MATLAB !!! FINAL VERSION !!!
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2023

close all
clear all

% declare global variables
%   s           serial port communication
%   hInput1     input widget 1
%   hInput1     input widget 2
%   hPlot       plot widget
%   hFig        figure widget
%   hTimer      continuous timer
%   c           command type
%   y1          data stream series 1
%   y2          data stream series 2
global s hInput1 hInput2 hPlot hFig hTimer c y1 y2 th1 th2 CornersX CornersY cid

%% Set up
% Create serial port object
s = serialport("COM3", 9600);
configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1)

% Create GUI
hFig = figure;

% Create input field for sending commands to microcontroller
hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25]);
hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);

% Create button for sending commands
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);
hSquare = uicontrol('Style', 'pushbutton', 'String', 'Square', 'Position', [40, 100, 200, 50], 'Callback', @sendSquare);

% Create plot area
hPlot = axes('Position', [0.2, 0.6, 0.6, 0.3]);

% Set up variables for real-time plotting
r=78;
c = [];
y1 = [];
y2 = [];
CornersX = [1 100 100   1  1];
CornersY = [1   1 100 100  1];
cid = 1;
t0 = now;

% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;


%% Callback function for sending commands
function sendCommand(~, ~)
global s hInput1 hInput2 r th1 th2

    % Get values from input fields as strings but convert to numbers
    input1 = str2num(get(hInput1, 'String'));
    input2 = str2num(get(hInput2, 'String'));
    
    % validate input
    if numel(input1) ~= 1
        return; % input field must contain one value
    end
    if numel(input2) ~= 1
        return; % input field must contain one value
    end
    
    % call IK here
    [theta1, theta2] = inverse_kinematics(input1, input2, 78);
    %th1 = 30;
    %th2 = 120;

    % format command values as a string, e.g. C40.0,3.5;
    cmdStr = sprintf("C%.2f,%.2f;", theta1, theta2);
    
    % Send command string to microcontroller
    write(s, cmdStr, "string");
end


%% Callback function fo reading time series values from microcontroller
function readDataTimer(~, ~)
global s hPlot c y1 y2 r cid

    % % Read the ASCII data from the serialport object.
    % dataStr = readline(s);
    % if isempty(dataStr)
    %     return;
    % end
    % if dataStr == ""
    %     return;
    % end
    % 
    % % Parse data values from string and add accumulate into arrays
    % % e.g. Arduino sending 2 series: c1,100
    % data = sscanf(dataStr, "%c,%f,%f");
    % 
    % %CALL FK HERE
    % cmd = data(1);
    % cth1 = data(2);
    % cth2 = data(3);
    % 
    % 
    % 
    % [cx, cy] = forward_kinematics(cth1, cth2, 78);
    % c = [c, data(1)];
    % y1 = [y1, cx]; 
    % y2 = [y2, cy];
    % 
    % % configure callback 
    % configureCallback(s, "off");
    % 
    % % real-time plotting, e.g. 2 series (y1, y2)
    % plot(hPlot, y1, y1, 'r-');
    % hold on
    % plot(hPlot, y1, y2, 'b-');
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
global s hFig hTimer

    % Stop timer
    stop(hTimer);
    delete(hTimer);
    
    % Close serial port
    %delete(s);
    
    % Close GUI
    delete(hFig);
end

%% Callback function for closing the GUI
function sendSquare(~, ~)
global s CornersX CornersY

for i = 1:length(CornersX)

    disp(i)
    
    input1 = CornersX(i);
    input2 = CornersY(i);
    
    % validate input
    if numel(input1) ~= 1
        return; % input field must contain one value
    end
    if numel(input2) ~= 1
        return; % input field must contain one value
    end
    
    % call IK here
    [theta1, theta2] = inverse_kinematics(input1, input2, 78)
    %th1 = 30;
    %th2 = 120;

    % format command values as a string, e.g. C40.0,3.5;
    cmdStr = sprintf("C%.2f,%.2f;", theta1, theta2);
    
    % Send command string to microcontroller
    write(s, cmdStr, "string")

    
    dataStr = readline(s)
    data = sscanf(dataStr, "%c,%f,%f")

    %CALL FK HERE
     cmd = data(1)
    % cth1 = data(2)
    % cth2 = data(3)
    while cmd == 99
        
        if cmd == 100
            break
        end
        
        dataStr = readline(s)
        data = sscanf(dataStr, "%c,%f,%f")
        cmd = data(1)
    
        %CALL FK HERE
        %cmd = data(1)
    end
    disp(i)
end
end
%end
