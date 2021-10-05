clear all; close all; clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%======================================  link 1
L1 = 0.5; % length [m]
c1 = L1/2; % mass center
m1 = 4.6; % mass [kg]
I1 = 1/12*m1*L1^2; % moment of inertia
%======================================  link 2
L2 = 0.3; % length [m]
c2 = L2/2; % mass center
m2 = 2.3; % mass [kg]
I2 = 1/12*m2*L2^2; % moment of inertia

g = 9.801; % gravity constant

tau = [5 2];
tau_G = [
ddtheta = [(tau(1))/I1 tau(2)/I2];

%% %%%%%%%%%%%%%%%%%% dynamic simulation
%%%%%%%%%%%%%%%%%%% discrete time
T = 2; % second
N = 200; % resolution
i = 0; 
for t = linspace(0, T, N)
    i = i + 1;
    time = t;
    dtheta(1,i) = ddtheta(1)*0.02*t; dtheta(2,i) = ddtheta(1)*0.02*t; 
    theta(1,i) = 1/2*ddtheta(1)*0.02*t^2; theta(2,i) = 1/2*ddtheta(2)*0.02*t^2;
    
    X1(i) = L1*cos(theta(1,i));
    Y1(i) = L1*sin(theta(1,i));
    Length(1,i) = X1(i)^2+Y1(i)^2;
    
    X2(i) = L2*cos(theta(1,i)+theta(2,i));
    Y2(i) = L2*sin(theta(1,i)+theta(2,i));
    Length(2,i) = X2(i)^2+Y2(i)^2;
    
    EEX = X1(i)+X2(i);
    EEY = Y1(i)+Y2(i);
end 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motion profiles
figure(2)
clf
figure(2)
subplot(2, 1, 1)
hold on
plot(time, theta(1,i), 'b')
plot(time, theta(2,i), 'r')
hold off
legend('joint 1', 'joint 2')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(2, 1, 2)
hold on
plot(time, dtheta(1,i), 'b')
plot(time, dtheta(2,i), 'r')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(2)
clf
figure(2)
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