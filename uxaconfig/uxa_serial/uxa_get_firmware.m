function [HardBreak, FirmwareVer] = uxa_get_firmware()
%Get firmware version of UXA-90 Serial Port
% Input: CMDGet_Firmware
% Output: Firmware version
% UXA-90 will return  17 hex value
global SerialPort
%%% Specify hex codes to be transmitted
% CMDGet_Firmware = ['ff';'ff';'aa';'55';'aa';'55';'37';'ba';'12';'01';'00';'00';'00';'01';'01';'01'];
CMDGet_Firmware = [255;255;170;85;170;85;55;186;18;1;0;0;0;1;1;1];

%Write using the UINT8 data format
uxa_serial_write(SerialPort,CMDGet_Firmware);
pause(0.05)
% Loop to get response firmware
Read_OK = false;
HardBreak = false;
ReSend_cnt = 0;
while ~Read_OK && ~HardBreak
    %%% Response Data: 17
    RespData_length = SerialPort.BytesAvailable;
    if RespData_length > 0
        RespData = fread(SerialPort,RespData_length,'uint8');
    if RespData_length == 17
        Read_OK = true;
        FirmwareVer = RespData(15:16)';        
    else
        % Request Firmware if read not succes
        uxa_serial_write(SerialPort,CMDGet_Firmware);
        pause(0.05)
        % Data is available but it is not response of Firmware request
        Read_OK = false;
        FirmwareVer = [];
        ReSend_cnt = ReSend_cnt + 1;
    end
    else
        % Request Firmware if read not succes
        uxa_serial_write(SerialPort,CMDGet_Firmware);
        pause(0.05)
        % No response data
        Read_OK = false;
        FirmwareVer = [];
        ReSend_cnt = ReSend_cnt + 1;        
    end
    if ReSend_cnt > 50
        HardBreak = true;
    end
end
end

