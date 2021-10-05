%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    The forward kinematics  of the 2-dof planar robotic arm
%  @AAU, Shaoping Bai 
%
% Two links are rotating with harmonic functions:
% theta_1 = theta_10+ A1*sin (f1*t);   theta_10 is the initial position
% theta_2 = theta_20+ A2*sin (f2*t);   theta_20 is the initial position 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc

%%  %%%%%%%%%%%%%%%%%%%%%%%  motion profiles
%=======================  joint 1
A1 = 3; % magnitude
f1 = 2; % frequency

%=======================  joint 2
A2 = 6; % magnitude
f2 = 1; % frequency



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
L1 = 0.5; % length [m]



%======================================  link 2
L2 = 0.3; % length [m]


theta_10=pi/3;theta_20=0; % initial angles of two joints


%%%%%%%%%%%%%%%%%%% discrete time
T = 4; % second
N = 100; % resolution
i = 0; 
for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t; 
    
    %%%%%%%%%%%%%%% Joint 1: angular displacement, velocity, acceleration
    theta_1(i) = theta_10+A1*sin(f1*t); dtheta_1(i) = A1*f1*cos(f1*t); 
    ddtheta_1(i) = -A1*f1^2*sin(f1*t); 
    
    %%%%%%%%%%%%%%% Joint 1: angular displacement, velocity, acceleration
    theta_2(i) = theta_20+A2*sin(f2*t); dtheta_2(i) = A2*f2*cos(f2*t);
    ddtheta_2(i) = -A2*f2^2*sin(f2*t); 
    Vtheta = [dtheta_1(i); dtheta_2(i)];
    Atheta = [ddtheta_1(i); ddtheta_2(i)];
    %%%%%%%%%%%%%%%%Jacobian
   J = [- L2*sin(theta_1(i) + theta_2(i)) - L1*sin(theta_1(i))  (-1)*L2*sin(theta_1(i) + theta_2(i)); 
            L2*cos(theta_1(i) + theta_2(i)) + L1*cos(theta_1(i))  L2*cos(theta_1(i) + theta_2(i))]; 
    %%%%%%%%%%%%%%%%%%% time derivative of Jacobian
        H = [ - L2*cos(theta_1(i) + theta_2(i))*sum(Vtheta) - L1*cos(theta_1(i))*Vtheta(1)  sum(Vtheta)*(-L2)*cos(theta_1(i) + theta_2(i)); 
        - L2*sin(theta_1(i) + theta_2(i))*sum(Vtheta) - L1*sin(theta_1(i))*Vtheta(1)  sum(Vtheta)*(-L2)*sin(theta_1(i) + theta_2(i))]; 
    
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link positions
    %%%%%%%%%%%%%%%%%% position of link 1 end
    X1(i) = L1*cos(theta_1(i)); Y1(i) = L1*sin(theta_1(i)); 
   
    %%%%%%%%%%%%%%%%%% position of link 2 end
    X2(i) = X1(i) + L2*cos(theta_1(i) + theta_2(i)); 
    Y2(i) = Y1(i) + L2*sin(theta_1(i) + theta_2(i)); 
 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% end-effector velocity
    
    Vee = J*Vtheta ; ve(:, i) = Vee;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% end-effector acceleration
    Aee = J*Atheta + H*Vtheta; ae(:, i) = Aee;
 
    
end

 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motion profiles
figure(2)
clf
figure(2)
subplot(3, 1, 1)
hold on
plot(time, theta_1, 'b')
plot(time, theta_2, 'r')
hold off
legend(' joint 1', 'joint 2')
grid on; 
xlabel('time [sec]'); ylabel('angular dis [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, ve(1, :), 'b')
plot(time, ve(2, :), 'r')
hold off
legend('v_x', 'v_y')
grid on; 
xlabel('time [sec]'); ylabel('ee vel. [m/s]'); 
subplot(3, 1, 3)
hold on
plot(time, ae(1, :), 'b')
plot(time, ae(2, :), 'r')
hold off
legend('a_x', 'a_y')
grid on; 
xlabel('time [sec]'); ylabel('ee acc. [m/s^2]'); 




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(3)
clf
figure(3)
hold on 
axis equal; grid on; axis([-1 1 -1 1]);
xlabel('x [m]'); ylabel('z [m]'); 
set(gca,'NextPlot','replaceChildren');
for j = 1 : N  
    plot(0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2)  
    plot(X1(j), Y1(j), 'bo', 'MarkerSize', 6, 'Linewidth', 2)
    plot(X2(j), Y2(j), 'ro', 'MarkerSize', 6, 'Linewidth', 2)
    line([0 X1(j)], [0 Y1(j)], 'Color', 'b', 'Linewidth', 2); 
    line([X1(j) X2(j)], [Y1(j) Y2(j)], 'Color', 'r', 'Linewidth', 2);  
    F(j) = getframe; 
end
hold off
movie(F); 