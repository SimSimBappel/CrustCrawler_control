  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%    The inverse kinematics of the 2-dof planar robotic arm
%    Contact: Shaoping Bai, e-mail: shb@m-tech.aau.dk
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  For any given end-effector trajectory, 
%  The programme calculates the joint angles, velocities and acceleration
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link length
%======================================  link 1
L1 = 0.5; %  length [m]

%======================================  link 2
L2 = 0.3; % length [m]

%% %%%%%%%%%%%%%%%%%% inverse kinematic simulation

%=============== end-effector position and velocity
x0 = 0.7; y0 = 0.1;  %starting position
Vx = -1.2; Vy = 0.5; Ve = [Vx; Vy]; % [m/s]

%%%%%%%%%%%%%%%%%%% discrete time
T = 1; % second
N = 100; % resolution
i = 0; 
for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t; 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  ee position
    x = Vx*t + x0; y = Vy*t + y0; p(:, i) = [x; y];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ee acceleration
    Aee = [0; 0];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  inverse geometric problem
    S = (x^2 + y^2)^(1/2);
    theta2 = pi - acos(((L1^2 + L2^2) - S^2)/(2*L1*L2));
    theta1 = atan2(y, x) - acos(((L1^2 + (x^2 + y^2) - L2^2))/(2*L1*S));
    theta(:, i) = [theta1; theta2];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  position of end: link 1 
    X1(i) = L1*cos(theta1); Y1(i) = L1*sin(theta1); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  position of end: link 2
    X2(i) = L1*cos(theta1) + L2*cos(theta1 + theta2); 
    Y2(i) = L1*sin(theta1) + L2*sin(theta1 + theta2); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Jacobian matrix
    J = [- L2*sin(theta1 + theta2) - L1*sin(theta1)  (-1)*L2*sin(theta1 + theta2); 
            L2*cos(theta1 + theta2) + L1*cos(theta1)  L2*cos(theta1 + theta2)]; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  joint angular velocity
    Vtheta = J\Ve; dtheta(:, i) = Vtheta; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  time derivative of Jacobian
    H = [ - L2*cos(theta1 + theta2)*sum(Vtheta) - L1*cos(theta1)*Vtheta(1)  sum(Vtheta)*(-L2)*cos(theta1 + theta2); 
        - L2*sin(theta1 + theta2)*sum(Vtheta) - L1*sin(theta1)*Vtheta(1)  sum(Vtheta)*(-L2)*sin(theta1 + theta2)]; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  joint angular acceleration
    Atheta = J\Aee - J\(H*Vtheta); ddtheta(:, i) = Atheta;
    
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motion profiles
figure(1)
clf
figure(1)
subplot(3, 1, 1)
hold on
plot(time, theta(1, :), 'b')
plot(time, theta(2, :), 'r')
hold off
legend('joint 1', 'joint 2')
grid on; 
xlabel('time [sec]'); ylabel('angular dis. [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, dtheta(1, :), 'b')
plot(time, dtheta(2, :), 'r')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular vel. [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, ddtheta(1, :), 'b')
plot(time, ddtheta(2, :), 'r')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acc. [rad/s^2]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(3)
clf
figure(3)
hold on 
grid on; axis([-1 1 -1 1]); axis equal; 
xlabel('x [m]'); ylabel('z [m]'); 
set(gca,'NextPlot','replaceChildren');
for j = 1 : N  
    plot(p(1, j), p(2, j), 'ko', 'MarkerSize', 6, 'Linewidth', 2)
    line([0 X1(j)], [0 Y1(j)], 'Color', 'b', 'Linewidth', 2); 
    line([X1(j) X2(j)], [Y1(j) Y2(j)], 'Color', 'r', 'Linewidth', 2);  
    line([p(1, 1) p(1, i)], [p(2, 1) p(2, i)], 'Color', 'g', 'Linewidth', 2);  
    F(j) = getframe; 
end
hold off
movie(F); 