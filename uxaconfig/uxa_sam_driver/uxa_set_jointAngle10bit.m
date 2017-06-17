function uxa_set_jointAngle10bit(samID,value)
global SerialPort
% Set Angle of SAM has ID samID a angle value 8 communitate
% Input: uxa_set_jointAngle
% Torq = 0 (max) : 254 (min)
% samID = 0:23
% value = 0:254

% Command Packet
CmdPacket = zeros(7,1);
binvalue = dec2bin(value,14);
CmdPacket(1) = 255; % Header 0xFF
CmdPacket(2) = bitshift(7,5,'uint8');
% CmdPacket(2) = bitor(bitshift(Torq,5,'uint8'),samID,'uint8');
CmdPacket(3) = 200; % C8
CmdPacket(4) = samID;
% CmdPacket(5) = Torq;
CmdPacket(5) = bin2dec(binvalue(1:7));
CmdPacket(6) = bin2dec(binvalue(8:14));
Checksum = bitxor(CmdPacket(2),CmdPacket(3),'uint8');
Checksum = bitxor(Checksum,CmdPacket(4),'uint8');
Checksum = bitxor(Checksum,CmdPacket(5),'uint8');
Checksum = bitxor(Checksum,CmdPacket(6),'uint8');
Checksum = bitand(Checksum,127,'uint8');
CmdPacket(7) = Checksum;
fwrite(SerialPort,CmdPacket,'uint8');
% 
% % RespData = fread(SerialPort,2,'uint8');
% RespData = fscanf(SerialPort);
end

