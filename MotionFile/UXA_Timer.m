function UXA_Timer
global tmr tmrcnt STOP
% crete a timer object and set properties
tmr = timer;
set(tmr, 'Period', 0.05);
set(tmr, 'ExecutionMode', 'FixedRate');
set(tmr, 'TimerFcn',{@uxatimerCallback});
start(tmr);
tmrcnt = 0;
STOP = false;
end

function uxatimerCallback(Obj, eventdata)
    global SamPos lendata lastSAMID tmrcnt STOP
    tmrcnt = tmrcnt + 1;
    if tmrcnt <= lendata
        for samID = 0:lastSAMID
            uxa_set_jointAngle(1,samID,round(SamPos(tmrcnt,samID+1)));
        end
    else
        STOP = true;
    end
end