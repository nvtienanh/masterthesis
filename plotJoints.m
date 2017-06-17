clear all
clc
close all
load 'SimData2UXA/RJoints.mat' 
load 'SimData2UXA/LJoints.mat'
load 'SimData2UXA/times.mat'

figure    
hold on 
plot(times,LJoints(:,1),'-k');  
plot(times,RJoints(:,1),'--k');   
grid on
legend('q_2','q_8');     
title('Yaw angle of hip (deg)');

figure    
hold on   
plot(times,LJoints(:,2),'-k');  
plot(times,RJoints(:,2),'--k'); 
grid on
legend('q_3','q_9');     
title('Roll angle of hip (deg)');

figure    
hold on 
plot(times,LJoints(:,3),'-k');  
plot(times,RJoints(:,3),'--k');   
grid on
legend('q_4','q_{10}');     
title('Pitch angle of hip (deg)');

figure    
hold on
   
plot(times,LJoints(:,4),'-k');  
plot(times,RJoints(:,4),'--k'); 
grid on
legend('q_5','q_{11}');     
title('Pitch angle of knee (deg)');

figure    
hold on
  
plot(times,LJoints(:,5),'-k');  
plot(times,RJoints(:,5),'--k');  
grid on
legend('q_6','q_{12}');     
title('Pitch angle of ankle (deg)');


figure    
hold on   
plot(times,LJoints(:,6),'-k');  
plot(times,RJoints(:,6),'--k'); 
grid on
legend('q_7','q_{13}');     
title('Roll angle of ankle (deg)');