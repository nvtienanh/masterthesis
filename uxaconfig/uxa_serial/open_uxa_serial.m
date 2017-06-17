clear all
close all
clc

% UXA_Serial connect
global SerialPort
% SerialPort=serial('/dev/ttyUSB0');% Linux
SerialPort = serial('COM6','BaudRate',115200,'Timeout',2);
set(SerialPort, 'Terminator', 'LF'); % Default terminator is \n
set(SerialPort,'DataBits',8);
set(SerialPort,'StopBits',1);

fopen(SerialPort);
SerialPort.ReadAsyncMode = 'continuous';
% uic_motion_cmd('pc_control');
% pause(2);
% flushinput(SerialPort);
% tic
% [firm_err,UXA_FirmwareVersion] = uxa_get_firmware();
% [seri_err,UXA_SerialNumber] = uxa_get_serialnumber();
% [zero_err,UXA_ZeroPosition] = uxa_get_zeroposition();
% toc
% close_uxa_serial;