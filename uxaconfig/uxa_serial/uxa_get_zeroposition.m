function [HardBreak,ZeroPosition] = uxa_get_zeroposition()
%Get firmware version of UXA-90 Serial Port
% Input: CMDGet_ZeroPosition
% Output: Firmware version
% UXA-90 will return  46 hex value
global SerialPort
%%% Specify hex codes to be transmitted
% ff ff aa 55 aa 55 37 ba 0b 01 00 00 00 01 01 01
% CMDGet_ZeroPosition = ['ff';'ff';'aa';'55';'aa';'55';'37';'ba';'0b';'01';'00';'00';'00';'01';'01';'01'];
CMDGet_ZeroPosition = [255;255;170;85;170;85;55;186;11;1;0;0;0;1;1;1];
%Convert to decimal format
% CMDGet_ZeroPosition = hex2dec(CMDGet_ZeroPosition);
%Write using the UINT8 data format
uxa_serial_write(SerialPort,CMDGet_ZeroPosition);
pause(0.05);

% Loop to get response firmware
Read_OK = false;
HardBreak = false;
ReSend_cnt = 0;
while ~Read_OK && ~HardBreak
    %%% Response Data: 46
    RespData_length = SerialPort.BytesAvailable;
    if RespData_length > 0
        RespData = fread(SerialPort,RespData_length,'uint8');
    if RespData_length == 46
        Read_OK = true;
        ZeroPosition = RespData(15:45)';     
    else
        % Request ZeroPosition if read not succes
        uxa_serial_write(SerialPort,CMDGet_ZeroPosition);
        pause(0.05)
        % Data is available but it is not response of Firmware request
        Read_OK = false;
        ZeroPosition = [];
        ReSend_cnt = ReSend_cnt + 1;
    end
    else
        % Request ZerosPosition if read not succes
        uxa_serial_write(SerialPort,CMDGet_ZeroPosition);
        pause(0.05)
        % No response data
        Read_OK = false;
        ZeroPosition = [];
        ReSend_cnt = ReSend_cnt + 1;        
    end
    if ReSend_cnt > 50
        HardBreak = true;
    end
end

end

