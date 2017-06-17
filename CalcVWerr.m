function err = CalcVWerr(Cref, Cnow)
global uLINK

perr = Cref.p - Cnow.p;
Rerr = Cnow.R^-1 * Cref.R;
werr = Cnow.R * rot2omega(Rerr);
% tmp = [Rerr(3,2)-Rerr(2,3);Rerr(1,3)-Rerr(3,1);Rerr(2,1)-Rerr(1,2)];
% 
% if isequal(Rerr,diag([1 1 1]))
%     ar = [0;0;0];
% elseif isequal(Rerr,diag([1 -1 -1])) || isequal(Rerr,diag([-1 1 -1])) || isequal(Rerr,diag([-1 -1 1]))
%     ar = pi/2.*[tmp(1)+1;tmp(2)+1;tmp(3)+1];
% else    
% ar = atan2(norm(tmp),Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1)/norm(tmp).*tmp;
% werr = 1/2*(cross(Cnow.R(:,1),Cref.R(:,1))+cross(Cnow.R(:,2),Cref.R(:,2)+cross(Cnow.R(:,3),Cref.R(:,3))));
% end
err = [perr; werr];
% err = [perr; ar];