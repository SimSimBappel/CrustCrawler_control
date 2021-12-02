clear;
clc;
% issym = @(x) isequal(x,x.')  
issym = @(x) isequal(x,x.');


syms ('q1', 'q2', 'q3', 'q4','real');
%Legths of links in mm
syms ('L1', 'L2', 'L3', 'L4','L5', 'real');

%Transformation
A01 = [cos(q1) 0 -sin(q1) L2*cos(q1);
       sin(q1) 0 cos(q1) L2*sin(q1);
       0  -1 0 L1;
       0 0 0 1];

A12 = [cos(q2-pi/2) -sin(q2-pi/2)  0 L3*cos(q2-pi/2);
       sin(q2-pi/2)  cos(q2-pi/2)  0 L3*sin(q2-pi/2);
       0  0 1 0;
       0 0 0 1];


A23 = [cos(q3+pi/2) 0 sin(q3+pi/2) -L4*cos(q3+pi/2);
       sin(q3+pi/2) 0 -cos(q3+pi/2) -L4*sin(q3+pi/2);
       0 1 0 0;
       0 0 0 1];

A34 = [cos(q4) -sin(q4) 0 0;
       sin(q4) cos(q4) 0 0;
       0 0 1 L5;
       0 0 0 1];

A02 = A01*A12;
A03 = A02*A23;
A04 = A03*A34;
A04 = simplify(A04);



%rotation matrices
R01= A01(1:3,1:3);
R12= A12(1:3,1:3);
R23= A23(1:3,1:3);
R34= A34(1:3,1:3);

%%  $$$$$$$$$$$$$$$
% Masses
syms('m1','m2','m3','m4','r1','r2','r3','r4','h1','h2','h3','h4','real');

%% 


%lengths :
% ri-1_i  =  the vector from joint i-1 to joint i expressed in i
% ri-1_ci= the vector from joint i-1 to the center of mass of link i
% ri_ci= the vector from joint i to the center of mass of link i expressed
% in i

r0_1= [L2 L1 0]'; % ri-1_i
r1_2=[L3 0 0]';
r2_3=[-L4 0 0]';
r3_4=[0 0 L5]';

r1_c1=[-L2/2 -L1/2 0]'; % ri_ci
r2_c2=[-L3/2 0 0]';
r3_c3=[L4/2 0 0]';
r4_c4=[0 0 -L5/2]';



%%%%%%%%%%%

syms('I1a','I1b','I1c','real');
I1(1,1)=I1a;
I1(2,2)=I1b;
I1(3,3)=I1c;


syms('I2a','I2b','I2c','real');
I2(1,1)=I2a;
I2(2,2)=I2b;
I2(3,3)=I2c;


syms('I3a','I3b','I3c','real');
I3(1,1)=I3a;
I3(2,2)=I3b;
I3(3,3)=I3c;


syms('I4a','I4b','I4c','real');
I4(1,1)=I4a;
I4(2,2)=I4b;
I4(3,3)=I4c;
 
  

z0=[0 0 1]';
% w0=0;
%w1=b1*q_d1;
% It is assumed that the base is downward and it is static. 
 
%% Forward
syms('q_d1','q_d2','q_d3','q_d4','real');
syms('q_dd1','q_dd2','q_dd3','q_dd4','real');

vec1=[q1,q2,q3,q4,q_d1,q_d2,q_d3,q_d4]; 
vec2=[q_d1;q_d2;q_d3;q_d4;q_dd1;q_dd2;q_dd3;q_dd4];
vec3=[q1,q2,q3,q4]; % q vector
vec4=[q_d1;q_d2;q_d3;q_d4]; % \dot{q} vector


% Link 1
syms('g_a','real');
g_a =9.8;   % grav accel
v_d0=[0 0 -g_a]'; %v0_0-g=[0 0 -g_a],  the manipulator is assumed to be downward. So, the "g" is downward, therefore it becomes minus in z direction. 


w1=R01'*[0 0 0]'+R01'*q_d1*z0;

w1_d=R01'*([0 0 0]'+cross(q_d1*[0 0 0]',z0)+q_dd1*z0);
w1_d_check=jacobian(w1,vec1)*vec2;

v1=R01'*v_d0 +  cross(w1, (cross(w1,r0_1 )))  + cross(w1_d,r0_1)  ; % siciliano as Pdd
v1c=v1+ cross(w1_d,r1_c1)+cross(w1, (cross(w1,r1_c1 )));

%% Link2
w2=R12'*(w1+q_d2*z0);

w2_d=R12'*(w1_d+cross(q_d2*w1,z0)+q_dd2*z0);
w2_d_check=jacobian(w2,vec1)*vec2;

v2=R12'*v1 +  cross(w2, (cross(w2,r1_2 )))  + cross(w2_d,r1_2)  ; % siciliano as Pdd
v2c=v2+ cross(w2_d,r2_c2)+cross(w2, (cross(w2,r2_c2 )));

%% Link 3
w3=R23'*(w2+q_d3*z0);

w3_d=R23'*(w2_d+cross(q_d3*w2,z0)+q_dd3*z0);
w3_d_check=jacobian(w3,vec1)*vec2;

v3=R23'*v2 +  cross(w3, (cross(w3,r2_3 )))  + cross(w3_d,r2_3)  ; % siciliano leei Pdd
v3c=v3+ cross(w3_d,r3_c3)+cross(w3, (cross(w3,r3_c3 )));

%% Link 4
w4=R34'*(w3+q_d4*z0);

w4_d=R34'*(w3_d+cross(q_d4*w3,z0)+q_dd4*z0);
w4_d_check=jacobian(w4,vec1)*vec2;

v4=R34'*v3 +  cross(w4, (cross(w4,r3_4 )))  + cross(w4_d,r3_4)  ; % siciliano as Pdd
v4c=v4+ cross(w4_d,r4_c4)+cross(w4, (cross(w4,r4_c4 )));

%%  Backward Recursion



% Link 4:
% terminal condition: 
% f5=tau5=0
% mass:

f5=[0; 0 ;0];
n5=[0 0 0]';
R45=eye(3);

f4=R45*f5+m4*v4c;
n4=cross(-f4,(r3_4+r4_c4))+R45*n5+cross((R45*f5),r4_c4)+I4*w4_d+ cross(w4,(I4*w4));
tau4=n4'*R34'*z0;

%% Link3
f3=R34*f4+m3*v3c;
n3=cross(-f3,(r2_3+r3_c3))+R34*n4+cross((R34*f4),r3_c3)+I3*w3_d+ cross(w3,(I3*w3));
tau3=n3'*R23'*z0;
%% Link 2:
f2=R23*f3+m2*v2c;
n2=cross(-f2,(r1_2+r2_c2))+R23*n3+cross((R23*f3),r2_c2)+I2*w2_d+ cross(w2,(I2*w2));
tau2=n2'*R12'*z0;
%% Link 1:
f1=R12*f2+m1*v1c;
n1=cross(-f1,(r0_1+r1_c1))+R12*n2+cross((R12*f2),r1_c1)+I1*w1_d+ cross(w1,(I1*w1));
tau1=n1'*R01'*z0;

tau=[tau1,tau2,tau3,tau4]';
tau=simplify(tau,1000);  % 

g1=subs(tau(1),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,0,0,0,0,0});
g2=subs(tau(2),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,0,0,0,0,0});
g3=subs(tau(3),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,0,0,0,0,0});
g4=subs(tau(4),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,0,0,0,0,0});

G=[g1;g2;g3;g4];
G=simplify(G,2000);


M11=subs(tau(1),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{1,0,0,0,0,0,0,0})-g1;
M12=subs(tau(1),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,1,0,0,0,0,0,0})-g1;
M13=subs(tau(1),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,1,0,0,0,0,0})-g1;
M14=subs(tau(1),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,1,0,0,0,0})-g1;

M21=subs(tau(2),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{1,0,0,0,0,0,0,0})-g2;
M22=subs(tau(2),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,1,0,0,0,0,0,0})-g2;
M23=subs(tau(2),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,1,0,0,0,0,0})-g2;
M24=subs(tau(2),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,1,0,0,0,0})-g2;

M31=subs(tau(3),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{1,0,0,0,0,0,0,0})-g3;
M32=subs(tau(3),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,1,0,0,0,0,0,0})-g3;
M33=subs(tau(3),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,1,0,0,0,0,0})-g3;
M34=subs(tau(3),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,1,0,0,0,0})-g3;

M41=subs(tau(4),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{1,0,0,0,0,0,0,0})-g4;
M42=subs(tau(4),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,1,0,0,0,0,0,0})-g4;
M43=subs(tau(4),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,1,0,0,0,0,0})-g4;
M44=subs(tau(4),{'q_dd1','q_dd2','q_dd3','q_dd4','q_d1','q_d2','q_d3','q_d4'},{0,0,0,1,0,0,0,0})-g4;


M=[M11,M12,M13,M14;
    M21,M22,M23,M24;
    M31,M32,M33,M34;
    M41,M42,M43,M44];
M=simplify(M,2000);

CD_dq_tmp=tau-M*[q_dd1;q_dd2;q_dd3;q_dd4]-G;
CD_dq=simplify(CD_dq_tmp,2000); % used additional argument 2000, should maybe be removed due to extreme time to run

CD_dq(1)=collect(CD_dq(1),q_dd1); 
CD_dq(1)=collect(CD_dq(1),q_dd2);
CD_dq(1)=collect(CD_dq(1),q_dd3);
CD_dq(1)=collect(CD_dq(1),q_dd4);


CD_dq(2)=collect(CD_dq(2),q_dd1);
CD_dq(2)=collect(CD_dq(2),q_dd2);
CD_dq(2)=collect(CD_dq(2),q_dd3);
CD_dq(2)=collect(CD_dq(2),q_dd4);


CD_dq(3)=collect(CD_dq(3),q_dd1);
CD_dq(3)=collect(CD_dq(3),q_dd2);
CD_dq(3)=collect(CD_dq(3),q_dd3);
CD_dq(3)=collect(CD_dq(3),q_dd4);

CD_dq(4)=collect(CD_dq(4),q_dd1);
CD_dq(4)=collect(CD_dq(4),q_dd2);
CD_dq(4)=collect(CD_dq(4),q_dd3);
CD_dq(4)=collect(CD_dq(4),q_dd4);

% Christoffel symboles Methods                          

for i = 1:4
    for j = 1:4
        for k = 1:4
        cmat(i,j,k) = 1/2 * ( diff(M(i,j), vec3(k)) + diff(M(i,k), vec3(j)) - diff(M(j,k), vec3(i)));
        end
    end
end

syms('C','real');
for i = 1:4
   
    for j = 1:4
       
        accum = 0;
        for k = 1:4
            accum = accum + cmat(i,j,k)* vec4(k);
        end
        C(i,j) = accum;
    end
    
end
C=simplify(C,2000);
er=CD_dq-C*vec4; % check? if er=0 so, IT IS OK!

% save('manipulator.mat','M','G','C')  %save the matrices M G and C to file





