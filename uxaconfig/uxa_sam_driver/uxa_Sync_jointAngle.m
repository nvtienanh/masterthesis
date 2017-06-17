function uxa_Sync_jointAngle(Torq,LastSamID,Angles)
global SerialPort
% Synchronized Position Move 8bit data
% Input: uxa_set_jointAngle
% Torq = 0 (max) : 4 (min)
% LastsamID 1: 32
% Angles = 1:254

% Command Packet
CmdLen = 5+LastSamID;
CmdPacket = zeros(CmdLen,1);
CmdPacket(1) = 255; % Header 0xFF
CmdPacket(2) = bitor(bitshift(Torq,5,'uint8'),31,'uint8'); %Data1
CmdPacket(3) = bitor(bitshift(0,5,'uint8'),LastSamID+1,'uint8'); %Data2
CmdPacket(4:CmdLen-1) = Angles;
% Calculate Checksum byte
Checksum = bitxor(CmdPacket(4),CmdPacket(5),'uint8');
for idx=6:LastSamID+1
    Checksum = bitxor(Checksum,CmdPacket(idx),'uint8');
end
Checksum = bitxor(Checksum,LastSamID+3,'uint8');
Checksum = bitand(Checksum,127,'uint8');
CmdPacket(CmdLen) = Checksum;
CmdPacket(CmdLen) =0;
display(CmdPacket);
% Send Package to UXA
fwrite(SerialPort,CmdPacket,'uint8');
% 
% % RespData = fread(SerialPort,2,'uint8');
% RespData = fscanf(SerialPort);
end

