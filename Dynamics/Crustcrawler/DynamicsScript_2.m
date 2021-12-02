%% setup
syms theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 ddtheta3
%theta1 = 100; theta2= 100; theta3 = 100; dtheta1 = 10; dtheta2 = 10; dtheta3 = 10;
LC1 = 0.0332; %Lengths form the joint to the COM in meters
LC2 = 0.1625;
LC3 = 0.132;

L1 = 0.0565; %Lengths form the previous joint to the next in meters
L2 = 0.2279;
L3 = 0.1335;

m1 = 0.226; %Mass of the link
m2 = 0.230;
m3 = 0.306;

g = [9.815; 0; 0]; 
z = [0; 0; 1];

s1 = [0; 0; L1];
s2 = [L2; 0; 0];
s3 = [L3; 0; 0];

sc2 = [LC2; 0; 0];
sc3 = [LC3; 0; 0];

%% Make Rotation matrices
a = [0;0;L2;L3];
alpha = [0;pi/2; pi/2;0];
d = [0;L1;0;0];

T01 = DHTrans(a(1),alpha(1),d(1),0,theta1);
T12 = DHTrans(a(2),alpha(2),d(2),0,theta2);
T23 = DHTrans(a(3),alpha(3),d(3),0,theta3);

R01 = T01(1:3,1:3);
R12 = T12(1:3,1:3);
R23 = T23(1:3,1:3);

%% Calculate velocities taken from craig s.176
omega0 = 0;
omegadot0 = 0;
Vdot0 = 0;


% omega1 = dtheta1*R01*[0;0;1];
% v1 = 0;
% 
% v2 = v1 + cross(omega1, s1);
% omega2 = omega1 + dtheta2*R12*z;
% vc2 = v2 + cross(omega2, sc2);
% 
% v3 = v2 + cross(omega2, s2);
% omega3 = omega2 + dtheta3*R23*z;
% vc3 = v3 + cross(omega3, sc3);

%% Inertia Tensor
% 
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
%               0.0114    0.0045    0.1627];
%% Calculate Lagrangian
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

%%

