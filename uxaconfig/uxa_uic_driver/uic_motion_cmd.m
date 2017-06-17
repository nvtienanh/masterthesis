function response = uic_motion_cmd(task )
% UXA_UIC_Driver motion commands
global SerialPort
Buffer = [255; 224; 225; 0; 0; 127];
    switch(task)
    case 'standup' 
      Buffer(5,:) = 7;      
      checksum = bitxor(Buffer(2),Buffer(3),'uint8');
      checksum = bitxor(checksum,Buffer(4),'uint8');
      checksum = bitxor(checksum,Buffer(5),'uint8');
      Buffer(6) = bitand(checksum,Buffer(6),'uint8');
      uxa_serial_write(SerialPort,Buffer);
      display('Standup Command');     
    case 'sitdown' 
      Buffer(5,:) = 21;      
      checksum = bitxor(Buffer(2),Buffer(3),'uint8');
      checksum = bitxor(checksum,Buffer(4),'uint8');
      checksum = bitxor(checksum,Buffer(5),'uint8');
      Buffer(6) = bitand(checksum,Buffer(6),'uint8');
      uxa_serial_write(SerialPort,Buffer);
      display('Sitdown Command');  
    case 'init' 
      Buffer(5,:) = 1;      
      checksum = bitxor(Buffer(2),Buffer(3),'uint8');
      checksum = bitxor(checksum,Buffer(4),'uint8');
      checksum = bitxor(checksum,Buffer(5),'uint8');
      Buffer(6) = bitand(checksum,Buffer(6),'uint8');
      uxa_serial_write(SerialPort,Buffer);
      display('Basic Posture');
    case 'pc_control' 
      Buff = [255;255;170;85;170;85;55;186;16;0;0;0;0;1;1;1];        
      uxa_serial_write(SerialPort,Buff);
      if SerialPort.BytesAvailable~=0
        response = fread(SerialPort,SerialPort.BytesAvailable,'uint8')';
      end
      display('PC Control');
    case 'direct_control' 
      Buff = [255;224;251;1;0;26];        
      uxa_serial_write(SerialPort,Buff);      
      if SerialPort.BytesAvailable~=0
        response = fread(SerialPort,SerialPort.BytesAvailable,'uint8')';
      end
      display('Direct Control');
    otherwise 
          
    end


end

