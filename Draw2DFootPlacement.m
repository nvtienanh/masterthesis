function Draw2DFootPlacement(p,wd,hi,op)
% Function DrawFoot of Biped robot
% fig = gcf;
% figure(fig);
plot([p(1)-wd/2 p(1)+wd/2],[p(2)+hi/2 p(2)+hi/2],op);
plot([p(1)+wd/2 p(1)+wd/2],[p(2)+hi/2 p(2)-hi/2],op);
plot([p(1)-wd/2 p(1)+wd/2],[p(2)-hi/2 p(2)-hi/2],op);
plot([p(1)-wd/2 p(1)-wd/2],[p(2)+hi/2 p(2)-hi/2],op);
% plot(p(1),p(2),'*');
end

