function [HardBreak,Load,Pos] = uxa_get_jointAngle(samID)
global SerialPort
% Set Angle of SAM has ID samID a angle value 8 communitate
% Input: uxa_set_jointAngle
% Torq = 0 (max) : 4 (min)
% samID = 0:23
% value = 0:254
uxa_set_jointAngle(5,samID,0);
% Response Data: Load | Position
% RespData = fread(SerialPort,2,'uint8');

% Loop to get response firmware
Read_OK = false;
HardBreak = false;
ReSend_cnt = 0;

while ~Read_OK && ~HardBreak
    %%% Response Data: 2
    RespData_length = SerialPort.BytesAvailable;
    if RespData_length > 0
        RespData = fread(SerialPort,RespData_length,'uint8');
        if RespData_length == 2
            Read_OK = true;
            Load = RespData(1);
            Pos = RespData(2);        
        else
            % ReSend request
            uxa_set_jointAngle(5,samID,0);
            % Data is available but it is not response of Firmware request
            Read_OK = false;
            Load = 255;
            Pos = 255;
            ReSend_cnt = ReSend_cnt + 1;
        end
    else
        % ReSend request
        uxa_set_jointAngle(5,samID,0);
        % No response data
        Read_OK = false;
        Load = 255;
        Pos = 255;
        ReSend_cnt = ReSend_cnt + 1;        
    end
    if ReSend_cnt > 500
        HardBreak = true;
    end
end

end

