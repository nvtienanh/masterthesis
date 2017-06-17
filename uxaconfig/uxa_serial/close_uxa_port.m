function close_uxa_port( uxaport)
%close_uxa_port Close uxa com port

stopasync(uxaport);
fclose(uxaport); % bad
delete(uxaport);
clear uxaport;

end

