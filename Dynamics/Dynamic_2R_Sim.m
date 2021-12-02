clear all; close all; clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
g = 9.801; % gravity constant
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

%%%%%%%%%%%%%%%%%%%%%%%  Inverse Kinematics
Lower_bound = -sqrt(((L1+L2)^2)/2);
Upper_bound = sqrt(((L1+L2)^2)/2);
End_Pose = (Upper_bound-Lower_bound).*rand(2,1) + Lower_bound;
 x = End_Pose(1); 
 y = End_Pose(2);

L3 = (x^2 + y^2)^(1/2);
theta2_f = pi - acos(((L1^2 + L2^2) - L3^2)/(2*L1*L2));
theta2_f2 = -pi + acos(((L1^2 + L2^2) - L3^2)/(2*L1*L2));
theta1_f = atan2(abs(y), x) - acos(((L1^2 + (x^2 + y^2) - L2^2))/(2*L1*L3));
phi_2 = asin(abs(y)/L3);
phi_1 = phi_2 + theta1_f(1);
theta1_f2 = 2*phi_1 + theta1_f(1);

%% Trajectory Planning
t_f = 4.5;
theta_s(1) = 0;
theta_s(2) = 0;

theta_prik_0 = 0;
theta_prik_f = 0;

a_0(1) = theta_s(1);
a_1(1) = theta_prik_0;
a_2(1) = 3/(t_f^2)*(theta1_f-theta_s(1))-2/(t_f)*theta_prik_0-1/(t_f)*theta_prik_f;
a_3(1) = -2/(t_f^3)*(theta1_f-theta_s(1))+1/(t_f^2)*(theta_prik_0+theta_prik_f);

a_0(2) = theta_s(2);
a_1(2) = theta_prik_0;
a_2(2) = 3/(t_f^2)*(theta2_f-theta_s(2))-2/(t_f)*theta_prik_0-1/(t_f)*theta_prik_f;
a_3(2) = -2/(t_f^3)*(theta2_f-theta_s(2))+1/(t_f^2)*(theta_prik_0+theta_prik_f);


%% %%%%%%%%%%%%%%%%%% dynamic simulation
%%%%%%%%%%%%%%%%%%% discrete time
T = t_f; % second
N = 100; % resolution
i = 0; 

for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t; 
    
    %Joint 1: angular displacement, velocity, acceleration in radians
    theta(1,i) = a_3(1)*t^3+a_2(1)*t^2+a_1(1)*t+a_0(1);
    dtheta(1,i) = 3*a_3(1)*t^2+2*a_2(1)*t+a_1(1); 
    ddtheta(1,i) = 2*3*a_3(1)*t+2*a_2(1); 
    
    % Joint 2: angular displacement, velocity, acceleration in radians
    theta(2,i) = a_3(2)*t^3+a_2(2)*t^2+a_1(2)*t+a_0(2);
    dtheta(2,i) = 3*a_3(2)*t^2+2*a_2(2)*t+a_1(2); 
    ddtheta(2,i) = 2*3*a_3(2)*t+2*a_2(2); 
    
    %Joint Positions
    X1(i) = L1*cos(theta(1,i));
    Y1(i) = L1*sin(theta(1,i));
    
    X2(i) = L1*cos(theta(1,i)) + L2*cos(theta(2,i)+theta(1,i));
    Y2(i) = L1*sin(theta(1,i)) + L2*sin(theta(2,i)+theta(1,i));
    length1(i) = X1(i)^2+Y2(i)^2;
    length2(i) = X2(i)^2+Y2(i)^2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%  coefficients of dynamic equation
    H11 = m1*c1^2 + I1 + m2*(L1^2 + c2^2 + 2*L1*c2*cos(theta(2,i))) + I2;
    H22 = m2*c2^2 + I2; 
    H12 = m2*(c2^2 + L1*c2*cos(theta(2,i))) + I2; 
    h = m2*L1*c2*sin(theta(2,i)); 
    G1 = m1*c1*g*cos(theta(1,i)) ...
        + m2*g*(c2*cos(theta(1,i) + theta(2,i)) + L1*cos(theta(1,i))); 
    G2 = m2*g*c2*cos(theta(1,i) + theta(2,i));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  joint angular velocity
      J(1,1) = -L1*sin(theta(1,i))-L2*sin(theta(2,i)+theta(1,i));
      J(1,2) = -L2*sin(theta(2,i)+theta(1,i));
      J(2,1) = L1*cos(theta(1,i)) + L2*cos(theta(2,i)+theta(1,i));
      J(2,2) = L2*cos(theta(2,i)+theta(1,i));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  time derivative of Jacobian
    H = [ - L2*cos(theta1_f + theta2_f)*(theta1_f + theta2_f) - L1*cos(theta1_f)*dtheta(1,i)  (theta1_f + theta2_f)*(-L2)*cos(theta1_f + theta2_f);
        - L2*sin(theta1_f + theta2_f)*(theta1_f + theta2_f) - L1*sin(theta1_f)*dtheta(1,i)  (theta1_f + theta2_f)*(-L2)*sin(theta1_f + theta2_f)];
   
    tau_1(i) = H11*ddtheta(1,i) + H12*ddtheta(2,i) - h*(dtheta(2,i))^2 ...
        -2*h*dtheta(1,i)*dtheta(2,i) + G1;
    tau_2(i) = H22*ddtheta(2,i) + H12*ddtheta(1,i) + h*(dtheta(1,i))^2 + G2;
    
        % Angular velocity and acceleration as vectors
    Vtheta = [dtheta(1,i); dtheta(2,i)];
    Atheta = [ddtheta(1,i); ddtheta(2,i)];
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% end-effector velocity and acceleration
    Vee = J*Vtheta ; ve(i) = sqrt(Vee(1)^2 +Vee(2)^2);
    Aee = J*Atheta + H*Vtheta; ae(i) = sqrt(Aee(1)^2 +Aee(2)^2);
 
    
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  plot the link
figure(1)
clf
figure(1)
hold on 
axis equal; grid on; axis([-1 1 -1 1]); 
xlabel('x [m]'); ylabel('z [m]'); 
set(gca,'NextPlot','replaceChildren');
for j = 1 : N  
    plot(0, 0, 'ko', 'MarkerSize', 6, 'Linewidth', 2)  
    plot(End_Pose(1), End_Pose(2), 'ko', 'MarkerSize', 6, 'Linewidth', 2)
    plot(X1(j), Y1(j), 'bo', 'MarkerSize', 6, 'Linewidth', 2)
    plot(X2(j), Y2(j), 'ro', 'MarkerSize', 6, 'Linewidth', 2)
    line([0 X1(j)], [0 Y1(j)], 'Color', 'b', 'Linewidth', 2); 
    line([X1(j) X2(j)], [Y1(j) Y2(j)], 'Color', 'r', 'Linewidth', 2);  
    F(j) = getframe; 
end
hold off
movie(F); 

%% 
figure(2)
clf
figure(2)
subplot(3, 1, 1)
hold on
plot(time, theta(1,:), 'b')
plot(time, theta(2,:), 'r')
hold off
legend(' joint 1', 'joint 2')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, dtheta(1,:), 'b')
plot(time, dtheta(2,:), 'r')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, ddtheta(1,:), 'b')
plot(time, ddtheta(2,:), 'r')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acceleration [rad/s^2]'); 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the motor torques
figure(3)
clf
figure(3)
hold on
plot(time, tau_1, 'b')
plot(time, tau_2, 'r')
hold off
legend(' joint 1', 'joint 2')
grid on; 
xlabel('time [sec]'); ylabel('torques [Nm/rad]'); 

%% End-effector placement, velocity and acceleration
figure(4)
clf
figure(4)
grid on; 
xlabel('time [sec]'); ylabel('Position [m]'); 
subplot(2, 1, 1)
hold on
plot(time, ve, 'b')
hold off
grid on; 
xlabel('time [sec]'); ylabel('End-Effector velocity [m/s]'); 
subplot(2, 1, 2)
hold on
plot(time, ae, 'b')
hold off
grid on; 
xlabel('time [sec]'); ylabel('End-Effector acceleration [m/s^2]'); 