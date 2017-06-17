%% Record Joint position
open_uxa_serial;
global tmr SamPos lendata lastSAMID STOP
DecodeMotionFile;
sam_config8bit;
lendata = length(SamPos);
lastSAMID = 11;

uic_motion_cmd('pc_control');
pause

uic_motion_cmd('standup');
pause

uic_motion_cmd('init');
pause

UXA_Timer;

while STOP == 0
    % loop until complete
    % walk 4 step    
end

stop(tmr)
delete(tmr)
close_uxa_serial;
clear 