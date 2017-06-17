function isComplete = uxa_serial_write( port,data )
%uxa_serial_write Summary of this function goes here
%   Detailed explanation goes here
for idx = 1:length(data)
    fwrite(port,data(idx),'uint8');  
%     pause(0.001);
end
isComplete = true;
end

