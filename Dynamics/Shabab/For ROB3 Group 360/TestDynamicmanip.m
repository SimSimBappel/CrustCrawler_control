clc
clear all
close all
dynamic_param;

dt=0.03; % sampling time
q=[  0          0.8    0.2    0.0]'; % initial joint configuration 
Dq=[ 0 0 0 0]'; %initial joint velocities


xe=FW_kin_MAN(q); 
FirstP=xe(1:6,1);


figure(1)
set(gca, 'Zdir', 'reverse', 'Xdir','reverse');
axis ([-1.5 1.5 -1.5 1.5 -1.5 1.5])
hold on
grid on
view(1,1)
zoom on;
zoom(3);
plot_manip(q,base,eye(3,3));
pause(0.5);


tau=[ 0.0 0.0 0.0 0.0]';  % torques at each joint (from servo)
d=1;
for i=1:500   
[Dq,q] = run_manip( Dq,tau,q,dt);




[xe] = FW_kin_MAN( q) ;
if (cos(xe(5,1))<=0.006)&(cos(xe(5,1))>=-0.006)
    fprintf('breake!');
  %  break;
end
pe(:,i)= xe;

figure(1)
cla;
if(i-d==1)
 d=i;
 plot_manip(q,base,eye(3,3));
DrawInertialFrame


plot3(xe(1),xe(2),xe(3),'.','Color','r') 
else 
plot3(xe(1),xe(2),xe(3),'.','Color','r') 
end
 pause(0.01);

    i=i+1;    
end

