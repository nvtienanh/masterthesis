function [CoM,dCoM,ZMPout]= calc_preview_control(ZMP_xy,Zcom,calc_time,preview_time,dt)

%     pc_time=1;          %Preview Width
%     calc_time=10;       %Time for Walk Pattern
    sample_time=dt;      %Sampling Time
    center_z=Zcom;       %Position of Center Of Gravity(z)
    g=-9.810;            %Acceleration Of Gravity

    Pre = (0:sample_time:preview_time)';
    foot_p_x = [ZMP_xy(:,1);repmat(ZMP_xy(end,1),size(Pre))];
    foot_p_y = [ZMP_xy(:,2);repmat(ZMP_xy(end,2),size(Pre))];
    
    %Coefficient Matrix of System
    A=[0 1 0;0 0 1;0 0 0];
    B=[0;0;1];
    C=[1 0 center_z/g];
    D=0;
    E_d=[sample_time;1;0];              
    sys=ss(A,B,C,D);                    
    sys_d=c2d(sys,sample_time);         
    [A_d,B_d,C_d,D_d]=ssdata(sys_d);    
    
    
    %Error System
    ZERO=[0;0;0];
    phi=[1 -C_d*A_d;ZERO A_d];
    G=[-C_d*B_d;B_d];
    GR=[1;ZERO];

    Q=zeros(4,4);   
    Q(1)=1e+7;      
    H=1;            

    %Riccati Equation
    [K,P]=dlqr(phi,G,Q,H);
    
    K=-(H+G'*P*G)^(-1)*G'*P*phi;

    xi = (eye(4,4)-G*(H+G'*P*G)^(-1)*G'*P)*phi;
    
    x = [0;0;0];
    y = [0;0;0];
    xp = x;
    yp = y;
    
    t = 0:sample_time:calc_time;
    
    %Initialized Increment Parameter
    count = 0;
    ux = 0;
    uy = 0;
    
    %Loop Start
    for tt = t
        
        count = count + 1;
        px = C_d * x;               %Output ZMP(x)
        py = C_d * y;               %Output ZMP(y)
        ex = foot_p_x(count) - px;  %Eroor between the target ZMP(x)
        ey = foot_p_y(count) - py;  %Eroor between the target ZMP(y)
        X = [ex ; x - xp];
        Y = [ey ; y - yp];
        xp = x;
        yp = y;
        
        dux = K * X;    %State Feedback
        j = 0;
        for ttt = tt : sample_time : (tt + preview_time-sample_time) ;
            j = j + 1;
            
            if (foot_p_x(count+j) - foot_p_x(count+j-1)) ~= 0                 
                f  = -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;         %ZMP feedforward term 
                dux = dux + f * (foot_p_x(count+j) - foot_p_x(count+j-1));  
            end
        end
        ux = ux + dux;      %Control input
        
        duy = K * Y;    %State Feedback
        j = 0;
        for ttt = tt : sample_time : (tt + preview_time-sample_time)
            j = j + 1;
            if (foot_p_y(count+j) - foot_p_y(count+j-1)) ~= 0
                f  = -(H+G'*P*G)^(-1)*G'*(xi')^(j-1)*P*GR;         %ZMP feedforward term
                duy = duy + f * (foot_p_y(count+j) - foot_p_y(count+j-1));
            end
        end
        uy = uy + duy;      %Control input
        
        x = A_d * x + B_d * ux;     %COG Trajectory(x)
        y = A_d * y + B_d * uy;     %COG Trajectory(y)
        
        com_x(count) = x(1);       %COG Trajectory
        com_y(count) = y(1);
        dcom_x(count) = x(2);       %COG Trajectory
        dcom_y(count) = y(2);
        zmp_x(count) = px;  %Output ZMP
        zmp_y(count) = py;        
    end
    com_z = repmat(Zcom,size(com_x));
    dcom_z = zeros(size(com_x));
    CoM    = [com_x' com_y' com_z'];
    dCoM   = [dcom_x' dcom_y' dcom_z'];
    ZMPout = [zmp_x' zmp_y'];
end
