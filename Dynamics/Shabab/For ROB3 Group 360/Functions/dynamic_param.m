%%
base=[0.0 0.0 0 0.0 0.0 0]'; 
x_v = 0.16; y_v = 0; z_v = 0.09;     % position of the manipulator base frame(0) relative to vehicle body-fixed frame(B)
phi_v = 0; th_v = 0; psi_v = 0;      % rpy-angles of the manipulator base frame(0) relative to vehicle body-fixed frame(B)


L1 = 77.8*10^-3; %meter
L2 = 2.2*10^-3;
L3 = 147.69*10^-3;
L4 = 28*10^-3;
L5 = 75.4*10^-3;



% mass of manipulator
 m1=0.2;
 m2=0.3;
 m3=0.2;
 m4=0.25;

% r= wide, h=height  of link

r1=5*10^-3;
r2=5*10^-3;
r3=6*10^-3;
r4=5*10^-3;
r5=5*10^-3;
h1=4*10^-3;
h2=4*10^-3;
h3=4*10^-3;
h4=4*10^-3;
h5 =4*10^-3;


 %inertia, moments
 
 
I1a=0.1;
I1b=0.1;
I1c=0.1;


I2a=0.1;
I2b=0.1;
I2c=0.1;


I3a=0.1;
I3b=0.1;
I3c=0.1;


I4a=0.1;
I4b=0.1;
I4c=0.1;




d0=1000;   % d0 = density of water, 1000 kg/m3


% linear skin-friction  % edw pairnoun tehtikes times.. stis eksiswseis
% exoun ginei arnitiko

Ds1_a=-0.1;
Ds1_b=-0.1;
Ds1_c=-0.1;


Ds2_a=-0.1;
Ds2_b=-0.1;
Ds2_c=-0.1;


Ds3_a=-0.1;
Ds3_b=-0.1;
Ds3_c=-0.1;


Ds4_a=-0.1;
Ds4_b=-0.1;
Ds4_c=-0.1;


Dd1_a=-0.1;
Dd1_b=-0.1;
Dd1_c=-0.1;


Dd2_a=-0.1;
Dd2_b=-0.1;
Dd2_c=-0.1;


Dd3_a=-0.1;
Dd3_b=-0.1;
Dd3_c=-0.1;


Dd4_a=-0.1;
Dd4_b=-0.1;
Dd4_c=-0.1;



% ROV  %dynamike eksiwseis tou LBV 
mv1 = 10-2;
mv2 = 10-3.5;
mv3 = 10-3.5;  
Iv1  =1.15;
Iv2 =1.25-0.4;
Iv3 =11.25-0.4;

Xu  = -4;
Xuu  =-12;
Yv  =-7;
Yvv =-21;
Zw  =-7;
Zww =-21;
Kp =-3;
Kpp =-9;
Mq =-3;
Mqq =-9;
Nr  =-3;
Nrr =-9; 
Bv = d0*0.8*0.36*0.36 ;
Wv = 10*9.8; 
zb = 0.1;

