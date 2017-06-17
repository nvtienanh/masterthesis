function [PGain,DGain] = uxa_set_PDGain(samID,PGain,DGain)
global SerialPort
% Get P and D Gain of SAM ID
CmdPacket = zeros(7,1);
CmdPacket(1) = 255; % Header 0xFF
CmdPacket(2) = 224; % E0 data1
CmdPacket(3) = 174; %0xAE data2
CmdPacket(4) = samID; % data3
CmdPacket(5) = PGain; % data4
CmdPacket(6) = DGain; % data4
% Checksum
checksum = bitxor(CmdPacket(2),CmdPacket(3),'uint8');
checksum = bitxor(checksum,CmdPacket(4),'uint8');
checksum = bitxor(checksum,CmdPacket(5),'uint8');
checksum = bitand(checksum,127,'uint8');
CmdPacket(7) = checksum;
fwrite(SerialPort,CmdPacket,'uint8');

%%% Response Data
RespDataSize = 2;
RespData = fread(SerialPort,RespDataSize,'uint8');
PGain = RespData(1); % Low Boundary
DGain = RespData(2); % Upp Boundary
end

