%%Forward Kinematics
%syms theta1 theta2 theta3;
theta1 = 100 
theta2 = 110
theta3 = 120

theta1 = pi/180 * theta1; 
theta2 = pi/180 * theta2;
theta3 = pi/180 * theta3;

 L1 = 0.0565;
 L2 = 0.222;
 L3 = 0.251;

% Transformations Matrices
T01 = [cos(theta1) -sin(theta1) 0 0;
       sin(theta1) cos(theta1)  0 0;
       0           0            1 0;
       0           0            0 1];
T12 = [cos(theta2) -sin(theta2) 0 0;
       0            0           -1 0;
       sin(theta2) cos(theta2)  0 L1;
       0           0            0 1];

T23 = transl(L2,0,0)*trotx(-pi/2)*trotz(theta3);

T3EE =[1 0 0 L3
       0 1 0 0
       0 0 1 0
       0 0 0 1];

  T02 = T01*T12;
  T03 = T02*T23;
  T0EE = T03*T3EE;
  T10 = inv(T01);
  T21 = inv(T12);
  T32 = inv(T23);
  T30 = inv(T03);    
  TEE0 = inv(T0EE);
  Pose = T0EE(1:3,4);
  %length = L2+L3-sqrt(Position(1)^2+Position(2)^2)
  
  %% Inverse kinematics
  [Pose_sph(1),Pose_sph(2),Pose_sph(3)] = cart2sph(Pose(1),Pose(2),Pose(3)-L1);
  Plane_pose_sph = [Pose_sph(1); pi/2; Pose_sph(3)];
  [Plane_pose_cart(1),Plane_pose_cart(2),Plane_pose_cart(3)] = sph2cart(Plane_pose_sph(1),0,Plane_pose_sph(3));
  x = cos(Plane_pose_sph(1))*Plane_pose_sph(3);
  y = sin(Plane_pose_sph(1))*Plane_pose_sph(3);
  z = cos(Plane_pose_sph(2))*Plane_pose_sph(3);
%   %% Plot the transformations
%   figure(1)
%   %plot3(Pose(1),Pose(2),Pose(3),'o');
%   plot3(x0,y0,z0,'o');
  %% 
  
x = Plane_pose_cart(1);
y = Plane_pose_cart(2);
r = (x^2 + y^2)^(1/2);

theta3_f = pi - acos(((L2^2 + L3^2) - r^2)/(2*L2*L3));
theta3_f2 = -pi + acos(((L2^2 + L3^2) - r^2)/(2*L2*L3));

theta2_f  = asin((Pose(3)-L1)/(111/500 + (251*cos(theta3_f))/1000));

if theta3_f > pi/2
% sometimes theta2_f has to be 
    theta2_f  = pi - theta2_f;
end

theta1_f = acos(1000*(251*cos(theta2_f)*cos(theta3_f)*Pose(1) + 222*cos(theta2_f)*Pose(1) + 251*sin(theta3_f)*Pose(2))/(63001*cos(theta2_f)^2*cos(theta3_f2)^2 + 111444*cos(theta2_f)^2*cos(theta3_f2) + 49284*cos(theta2_f)^2 + 63001*sin(theta3_f)^2));

% if theta3_f > pi/2 |  2*pi+theta3_f2 > 4*pi/3
% theta1_f = 2*pi-acos(1000*(251*cos(theta2_f)*cos(theta3_f2)*Pose(1) + 222*cos(theta2_f)*Pose(1) + 251*sin(theta3_f2)*Pose(2))/(63001*cos(theta2_f)^2*cos(theta3_f2)^2 + 111444*cos(theta2_f)^2*cos(theta3_f2) + 49284*cos(theta2_f)^2 + 63001*sin(theta3_f2)^2));
% theta3_f = theta3_f2;
% end


theta1 = theta1_f*(180/pi)
theta1_2 = theta1_f2*(180/pi);
theta2 = theta2_f*(180/pi)
theta3 = theta3_f*(180/pi)
