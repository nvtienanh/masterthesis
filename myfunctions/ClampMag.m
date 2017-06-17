function out = ClampMag( w ,d)
%ClampMag Summary of this function goes here
%   Setting target positions closer.

if norm(w)<=d
    out = d;
else
    out = d*w/norm(w);
end

end

