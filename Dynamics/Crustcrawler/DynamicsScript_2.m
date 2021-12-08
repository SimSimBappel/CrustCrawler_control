%% setup
clear
syms theta1 theta2 theta3 dtheta1 dtheta2 dtheta3 ddtheta1 ddtheta2 ddtheta3 mObj
%theta1 = 100; theta2= 100; theta3 = 100; dtheta1 = 10; dtheta2 = 10; dtheta3 = 10;

L1 = 0.0565; %Lengths form the previous joint to the next in meters
L2 = 0.2279;
L3 = 0.1335;

s1 = [0; 0; L1]; %vectors from the previous joint to the next in meters
s2 = [L2; 0; 0];
s3 = [L3; 0; 0];

LC1 = 0.0332; %Lengths form the joint to the COM in meters
LC2 = 0.1625;
LC3 = 0.132;

sc1 = [0; 0; LC1]; %Vectors form the joint to the COM in meters
sc2 = [LC2; 0; 0];
sc3 = [LC3; 0; 0];

m1 = 0.226; %Mass of the links
m2 = 0.230;
m3 = 0.306;

g = [9.815; 0; 0]; 
z = [0; 0; 1];

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

I1 = sym('I1', [3 3]);
I2 = sym('I2', [3 3]);
I3 = sym('I3', [3 3]);


%% Make Rotation matrices from DH-parameters
a = [0;0;L2;L3];
alpha = [0;pi/2; pi/2;0];
d = [0;L1;0;0];

T01 = DHTrans(a(1),alpha(1),d(1),0,theta1);
T12 = DHTrans(a(2),alpha(2),d(2),0,theta2);
T23 = DHTrans(a(3),alpha(3),d(3),0,theta3);

R01 = T01(1:3,1:3);
R12 = T12(1:3,1:3);
R23 = T23(1:3,1:3);
R03 = R01*R12*R23;

R10 = transpose(R01);
R21 = transpose(R12);
R32 = transpose(R23);
R30 = transpose(R03);

%% Outward Iteration taken from Craig p.176
omega0 = zeros(3,1);
omegadot0 = zeros(3,1) ;
vdot0 = -g;
vcdot0 = vdot0;
F0 = zeros(3,1);
N0= zeros(3,1);

%joint 1
omega1 = zeros(3,1) + dtheta1*z;
omegadot1 = zeros(3,1) + cross(zeros(3,1),dtheta1*z)+ddtheta1*z;
vdot1 = R10*(cross(omega0,zeros(3,1))+cross(omega0,cross(omega0,zeros(3,1)))+vdot0);
vcdot1 = cross(omega1,sc1)+cross(omega1,cross(omega1,sc1)+vdot1);
F1 = m1*vcdot1;
N1= I1*omegadot1+cross(omega1,I1*omega1);

%joint 2 
omega2 = R21*omega1+dtheta2*z;
omegadot2 = R21*omegadot1+ cross(R21*omega1,dtheta2*z)+ddtheta2*z;
vdot2 = R21*(cross(omega1,sc2)+cross(omega1,cross(omega1,s1))+vdot1);
vcdot2 = cross(omega2,sc2)+cross(omega2,cross(omega2,sc2)+vdot2);
F2 = m2*vcdot2;
N2= I2*omegadot2+cross(omega2,I2*omega2);

%joint 3 
omega3 = R32*omega2+dtheta3*z;
omegadot3 = R32*omegadot2+ cross(R32*omega2,dtheta3*z)+ddtheta3*z;
vdot3 = R32*(cross(omega2,sc3)+cross(omega2,cross(omega2,s2))+vdot2);
vcdot3 = cross(omega3,sc3)+cross(omega3,cross(omega3,sc3)+vdot3);
F3 = m3*vcdot3;
N3= I3*omegadot3+cross(omega3,I3*omega3);

%% Inward Iteration from Craig p. 176
fObj = mObj*R03*g;

%Joint 3
f3 = eye(3)*fObj+F3;
n3 = N3 + eye(3)*cross(fObj,s3) + cross(sc3,F3) + cross(s3,eye(3)*F3);
tau3 = transpose(n3)*z;

%Joint 2
f2 = R23*f3+F2;
n2 = N2 + R23*cross(fObj,s2) + cross(sc2,F2) + cross(s2,R23*F2);
tau2 = transpose(n2)*z;

%Joint 1
f1 = R12*f2+F1;
n1 = N1 + R12*cross(fObj,s1) + cross(sc1,F1) + cross(s1,R12*F1);
tau1 = transpose(n1)*z;

%% Calculate Lagrangian

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

%% Gravity Calculation
g_3 = R30*g;
tau_g = cross(g_3,sc3)
