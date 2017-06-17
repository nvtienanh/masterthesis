function ShowRobot()
%%%  ShowRobot.m
% % ShowRobot model
    close(findobj('type','figure','name','showrobotleft'));
    close(findobj('type','figure','name','showrobotright'));
    fig_1 = figure('Name','showrobotleft');    
    movegui(fig_1,'northwest');
    hold on
    axis equal
    com = calcCoM;
    h(1) = Draw3DBall(0.03,com);% Drawing CoM
    DrawAllJoints(1);    
    tempx = xlim;
    tempy = ylim;
    tempz = zlim;
    axis manual
    axis([tempx*1.2 tempy*1.2 -0.2 1.5])
    daspect([1 1 1])
    arrow3([tempx(1) 0 0],[tempx(2) 0 0],'-r1.0',1);
    arrow3([0 tempy(1) 0],[0 tempy(2) 0],'-g1.0',1);
    arrow3([0 0 tempz(1)],[0 0 tempz(2)*1.3],'-b1.0',1);    
    grid on
    text(tempx(2),0,0,'X','FontWeight','bold','color','r');
    text(0,tempy(2),0,'Y','FontWeight','bold','color','g');
    text(0,0,tempz(2)*1.3,'Z','FontWeight','bold','color','b');
    view([0 0]);
    hold off
    fig_2 = figure('Name','showrobotright');    
    movegui(fig_2,'northeast');
    hold on
    axis equal
    h(2) = Draw3DBall(0.03,com);% Drawing CoM
    DrawAllJoints(1);    
    axis manual
    axis([tempx*1.2 tempy*1.2 -0.2 1.5])
    daspect([1 1 1])
    arrow3([tempx(1) 0 0],[tempx(2) 0 0],'-r1.0',1);
    arrow3([0 tempy(1) 0],[0 tempy(2) 0],'-g1.0',1);
    arrow3([0 0 tempz(1)],[0 0 tempz(2)*1.3],'-b1.0',1);    
    grid on
    text(tempx(2),0,0,'X','FontWeight','bold','color','r');
    text(0,tempy(2),0,'Y','FontWeight','bold','color','g');
    text(0,0,tempz(2)*1.3,'Z','FontWeight','bold','color','b');
    view([90 0]);
end

