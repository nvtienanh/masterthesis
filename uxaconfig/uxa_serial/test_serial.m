%%% Test Serial Communication with UXA
%%% author: nvtienanh
%%% web: nvtienanh.com
clear
clc
close all
global SerialPort
SerialPort = serial('COM5','BaudRate',115200,'Timeout',1);
set(SerialPort, 'Terminator', 'LF'); % Default terminator is \n
set(SerialPort,'DataBits',8);
set(SerialPort,'StopBits',1);


plotTitle = 'Serial Data Log';  % plot title
xLabel = 'Elapsed Time (s)';    % x-axis label
yLabel = 'Data';                % y-axis label
plotGrid = 'on';                % 'off' to turn off grid
min = -1.5;                     % set y-min
max = 1.5;                      % set y-max
scrollWidth = 10;               % display period in plot, plot entire data log if <= 0
 
%Define Function Variables
time = 0;
data = 0;
count = 0;
 
%Set up Plot
plotGraph = plot(time,data,'-mo',...
                'LineWidth',1,...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor',[.49 1 .63],...
                'MarkerSize',2);
             
title(plotTitle,'FontSize',25);
xlabel(xLabel,'FontSize',15);
ylabel(yLabel,'FontSize',15);
axis([0 10 min max]);
grid(plotGrid);

fopen(SerialPort);
SerialPort.ReadAsyncMode = 'continuous';
% readasync(SerialPort);
MPU_Connect_OK = true;

% uic_motion_cmd('pc_control');

%%% Specify hex codes to be transmitted
% CMDGet_Firmware = ['ff';'ff';'aa';'55';'aa';'55';'37';'ba';'12';'01';'00';'00';'00';'01';'01';'01'];
CMDGet_Firmware = [255;255;170;85;170;85;55;186;18;1;0;0;0;1;1;1];
%Write using the UINT8 data format
uxa_serial_write(SerialPort,CMDGet_Firmware);

% while ishandle(plotGraph) %Loop when Plot is Active 
% %     str = fscanf(SerialPort, 'uint8',4); 
%     aa = SerialPort.BytesAvailable;
%     drawnow;    
% end
% time_exe = toc;
% if (MPU_Connect_OK || ~ishandle(plotGraph))
%     stopasync(SerialPort);
%     fclose(SerialPort); % bad
%     delete(SerialPort);
%     clear SerialPort;
% end
% 
% clear count max min plotGraph plotGrid plotTitle ...
%         scrollWidth xLabel yLabel; 
%  
% disp('Session Terminated...');

