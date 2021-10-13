syms theta1 theta2 theta3;
% theta1=0; theta2= 0; theta3 = 0;
h12 = 0.0565;
h23 = 0.222;
h3EE = 0.251;

% Transformations Matrices
T01 = [cos(theta1) -sin(theta1) 0 0;
       sin(theta1) cos(theta1)  0 0;
       0           0            1 0;
       0           0            0 1];
T12 = [cos(theta2) -sin(theta2) 0 0;
       0           1            0 0;
       sin(theta2) cos(theta2)  0 h12;
       0           0            0 1];
   
T23 = [cos(theta3) 0 -sin(theta3) h23;
      sin(theta3) 0 cos(theta3)  0;
      0           1 0            0;
      0           0 0            1];  
T3EE =[1 0 0 h3EE
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
  Position = T0EE(1:3,4)