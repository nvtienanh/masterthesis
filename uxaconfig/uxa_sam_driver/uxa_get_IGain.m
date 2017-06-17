function IGain = uxa_get_IGain(samID)
global SerialPort
% Get I Gain of SAM ID

CmdPacket = zeros(7,1);
CmdPacket(1) = 255; % Header 0xFF
CmdPacket(2) = 224; % data1
CmdPacket(3) = 182; %0x0A data2
CmdPacket(4) = samID; % data3
CmdPacket(5) = 0; % data4
CmdPacket(6) = 0; % data4
% Checksum
checksum = bitxor(CmdPacket(2),CmdPacket(3),'uint8');
checksum = bitxor(checksum,CmdPacket(4),'uint8');
checksum = bitxor(checksum,CmdPacket(5),'uint8');
checksum = bitand(checksum,127,'uint8');
CmdPacket(7) = checksum;
fwrite(SerialPort,CmdPacket,'uint8');

%%% Response Data
RespDataSize = 1;
IGain = fread(SerialPort,RespDataSize,'uint8');
end

