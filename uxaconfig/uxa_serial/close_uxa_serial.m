global SerialPort

stopasync(SerialPort);
fclose(SerialPort); % bad
delete(SerialPort);
clear SerialPort;
% delete(instrfindall)

