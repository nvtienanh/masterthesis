function uxaport = open_uxa_port()
%init_uxa_serialport Setup Serial Port of UXA Robot

SerialPort = serial('COM6','BaudRate',115200,'Timeout',2);
set(SerialPort, 'Terminator', 'LF'); % Default terminator is \n
set(SerialPort,'DataBits',8);
set(SerialPort,'StopBits',1);
fopen(SerialPort);
SerialPort.ReadAsyncMode = 'continuous';
uxaport = SerialPort;

end

