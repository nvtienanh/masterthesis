function Angle = uxa_get_jointAngle10bit(samID)
global SerialPort
% Set Angle of SAM has ID samID a angle value 8 communitate
% Input: uxa_set_jointAngle
% Torq = 0 (max) : 254 (min)
% samID = 0:23
% value = 0:254

% Command Packet
CmdPacket = zeros(7,1);
CmdPacket(1) = 255; % Header 0xFF
CmdPacket(2) = bitshift(7,5,'uint8');
CmdPacket(3) = 173; % AD
CmdPacket(4) = samID;
% CmdPacket(5) = Torq;
CmdPacket(5) = 0;
CmdPacket(6) = 0;
Checksum = bitxor(CmdPacket(2),CmdPacket(3),'uint8');
Checksum = bitxor(Checksum,CmdPacket(4),'uint8');
Checksum = bitxor(Checksum,CmdPacket(5),'uint8');
Checksum = bitxor(Checksum,CmdPacket(6),'uint8');
Checksum = bitand(Checksum,127,'uint8');
CmdPacket(7) = Checksum;
fwrite(SerialPort,CmdPacket,'uint8');
% 
Response = fread(SerialPort,2,'uint8');
H7 = dec2bin(Response(1),7);
L7 = dec2bin(Response(2),7);
Angle = bin2dec([H7 L7]);

end

