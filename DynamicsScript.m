syms theta1(t) theta2(t) theta3(t) L1 L2 L3 LC1 LC2 LC3 LCE m1 m2 m3 mEE g h1 h2 h3 hEE;
dtheta1 = diff(theta1, t);
dtheta2 = diff(theta2, t);
dtheta3 = diff(theta3, t);
omega1 = dtheta1*[0;0;1];
z = [0;0;1];
v1 = 0;
s1 = [0;0;0];
v2 = v1 + cross(omega1, s1);
R01 = rotz(theta1);
R12 = [cos(theta2), -sin(theta2), 0; 
        0,           0,           1;
        sin(theta2), cos(theta2), 0];
    
R23= [cos(theta3), 0, -sin(theta3);
      sin(theta3), 0,  cos(theta3);
      0,           1,           0];
R02 = R01*R12;
R03 = R02*R23;
s2 = [0;0;L1];
sc2 = s2 + R02*[LC2;0;0];
omega2 = omega1 + dtheta2*R02*z;
vc2 = v2 + cross(omega2, sc2);
v3 =v2 + cross(omega2, s2);
omega3 = omega2 + dtheta3*R03*z;
sc3 = s1 + s2 + [LC3*cos(theta3); 0; LC3*sin(theta3)];
vc3 = v3 + cross(omega3, sc3);
sEE = s1 + s2 + [L3*cos(theta3); 0; L3*sin(theta3)];
vEE = v3 + cross(omega3, sEE);
I1= [0,0,0;
     0,0,0
     0,0,0];
 I2= [0,0,0;
     0,0,0
     0,0,0];
 I3= [0,0,0;
     0,0,0
     0,0,0];
 IEE= [0,0,0;
     0,0,0
     0,0,0];
T1 = 1/2*dot(omega1, I1*omega1);
V1 = m1*g*h1;
T2 = 1/2*m2*dot(vc2, vc2) + 1/2*dot(omega2,I2*omega2);
V2 = m2*g*h2;
T3 = 1/2*m3*dot(vc3, vc3) + 1/2*dot(omega3, I3*omega3);
V3 = m3*g*h3;
L = T1 - V1 + T2 - V2 + T3 - V3;
Torque1 = diff(diff(L, dtheta1),t)- diff(L, theta1);
Torque2 = diff(diff(L, dtheta2),t)- diff(L, theta2);
Torque3 = diff(diff(L, dtheta3),t)- diff(L, theta3);

