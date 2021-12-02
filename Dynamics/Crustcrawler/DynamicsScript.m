%syms theta1(t) theta2(t) theta3(t); 
%% setup
syms theta1 theta2 theta3 dtheta1 dtheta2 dtheta3
%theta1 = 100; theta2= 100; theta3 = 100; dtheta1 = 10; dtheta2 = 10; dtheta3 = 10;
LC1 = 0.0332;
LC2 = 0.1625;
LC3 = 0.132;

L1 = 0.0565;
L2 = 0.2279;
L3 = 0.1335;

m1 = 0.226;
m2 = 0.230;
m3 = 0.306;
%% Calculate velocities

omega1 = dtheta1*[0;0;1];
g = [9.815;0;0];
z = [0;0;1];
v1 = 0;
s1 = [0;0;5.65];
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
s2 = [L2;0;0];
sc2 = s1 + R02*[LC2;0;0];
omega2 = omega1 + dtheta2*R02*z;
vc2 = v2 + cross(omega2, sc2);
v3 =v2 + cross(omega2, s2);
omega3 = omega2 + dtheta3*R03*z;
sc3 = s1 + s2 + [LC3*cos(theta3); 0; LC3*sin(theta3)];
vc3 = v3 + cross(omega3, sc3);
sEE = s1 + s2 + [L3*cos(theta3); 0; L3*sin(theta3)];
vEE = v3 + cross(omega3, sEE);
%% Calculate Lagrangian

% I2= 1.0e-03 * [0.7074   -0.0000   -0.0018;
%               -0.0000    0.7021    0.0087;
%               -0.0018    0.0087    0.0577]; 
%           
% 
% I1= 1.0e-03 * [0.1315    0.0031    0.0014;
%                0.0031    0.0953    0.0002;
%                0.0014    0.0002    0.1544];
% 
% 
% I3= 1.0e-03 * [0.7270    0.0001    0.0114;
%                0.0001    0.6321    0.0045;
%                0.0114    0.0045    0.1627];

I1 = sym('I1', [3 3]);
I2 = sym('I2', [3 3]);
I3 = sym('I3', [3 3]);

h1 = [0;0;5.65]+[L2+LC3;0;0];
h2 = R01*s1 + R02*sc2 + [L2-LC2;0;0] + [LC3;0;0]; 
h3 = R01*s1 + R02*sc2 + R03*sc3;
T1 = 1/2*dot(omega1, I1*omega1);
V1 = m1*dot(g,h1);

T2 = 1/2*m2*dot(vc2, vc2) + 1/2*dot(omega2,I2*omega2);

V2 = m2*dot(g,h2);
T3 = 1/2*m3*dot(vc3, vc3) + 1/2*dot(omega3, I3*omega3);
V3 = m3*dot(g,h3);

%L = T1 - V1(1) + T2 - V2(1) + T3 - V3(1)


