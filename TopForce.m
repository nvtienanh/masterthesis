function [f,t] = TopForce(j)
global uLINK G
w_c = uLINK(j).R * uLINK(j).c + uLINK(j).p;   % ????
f = [0 0 -uLINK(j).m * G]';    % ??
t = cross(w_c, f);            % ??????????????

if uLINK(j).p(3) < 0.0  % ????????????
    Kf = 1.0E+4;        % ?????[N/m]
    Df = 1.0E+3;        % ?????[N/(m/s)]
    v = uLINK(j).vo + cross(uLINK(j).w,uLINK(j).p);  % ?????
    fc = [-Df*v(1)  -Df*v(2) -Kf*uLINK(j).p(3)-Df*v(3)]';
    f = f + fc;
    t = t + cross(uLINK(j).p, fc);
end
