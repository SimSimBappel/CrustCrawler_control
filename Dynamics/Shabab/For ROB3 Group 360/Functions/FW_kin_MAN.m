function [xe]=FW_kin_MAN(q)


q1=q(1); %rad
q2=q(2);
q3=q(3);
q4=q(4);


L1 = 77.8*10^-3; %meter
L2 = 2.2*10^-3;
L3 = 147.69*10^-3;
L4 = 28*10^-3;
L5 = 75.4*10^-3;


A0E =[ - sin(q1)*sin(q4) - cos(q4)*(cos(q1)*sin(q2 - pi/2)*sin(pi/2 + q3) - cos(q1)*cos(q2 - pi/2)*cos(pi/2 + q3)), sin(q4)*(cos(q1)*sin(q2 - pi/2)*sin(pi/2 + q3) - cos(q1)*cos(q2 - pi/2)*cos(pi/2 + q3)) - cos(q4)*sin(q1), cos(q1)*cos(q2 - pi/2)*sin(pi/2 + q3) + cos(q1)*cos(pi/2 + q3)*sin(q2 - pi/2), L5*(cos(q1)*cos(q2 - pi/2)*sin(pi/2 + q3) + cos(q1)*cos(pi/2 + q3)*sin(q2 - pi/2)) + L2*cos(q1) + L3*cos(q1)*cos(q2 - pi/2) - L4*cos(q1)*cos(q2 - pi/2)*cos(pi/2 + q3) + L4*cos(q1)*sin(q2 - pi/2)*sin(pi/2 + q3);
   cos(q1)*sin(q4) + cos(q4)*(cos(q2 - pi/2)*cos(pi/2 + q3)*sin(q1) - sin(q1)*sin(q2 - pi/2)*sin(pi/2 + q3)), cos(q1)*cos(q4) - sin(q4)*(cos(q2 - pi/2)*cos(pi/2 + q3)*sin(q1) - sin(q1)*sin(q2 - pi/2)*sin(pi/2 + q3)), cos(q2 - pi/2)*sin(q1)*sin(pi/2 + q3) + cos(pi/2 + q3)*sin(q1)*sin(q2 - pi/2), L5*(cos(q2 - pi/2)*sin(q1)*sin(pi/2 + q3) + cos(pi/2 + q3)*sin(q1)*sin(q2 - pi/2)) + L2*sin(q1) + L3*cos(q2 - pi/2)*sin(q1) - L4*cos(q2 - pi/2)*cos(pi/2 + q3)*sin(q1) + L4*sin(q1)*sin(q2 - pi/2)*sin(pi/2 + q3);
                                    -cos(q4)*(cos(q2 - pi/2)*sin(pi/2 + q3) + cos(pi/2 + q3)*sin(q2 - pi/2)),                                   sin(q4)*(cos(q2 - pi/2)*sin(pi/2 + q3) + cos(pi/2 + q3)*sin(q2 - pi/2)),                 cos(q2 - pi/2)*cos(pi/2 + q3) - sin(q2 - pi/2)*sin(pi/2 + q3),                                                 L1 + L5*(cos(q2 - pi/2)*cos(pi/2 + q3) - sin(q2 - pi/2)*sin(pi/2 + q3)) - L3*sin(q2 - pi/2) + L4*cos(q2 - pi/2)*sin(pi/2 + q3) + L4*cos(pi/2 + q3)*sin(q2 - pi/2);
                                                                                                           0,                                                                                                         0,                                                                             0,                                                                                                                                                                                                                 1];
                                                                                                       
                                                                                                       
                                                                                                       
xe(1:3,1)=A0E(1:3,4);

[rx ry rz]= GetEulerAngles(A0E(1:3,1:3));

xe(4,1)=rx;
xe(5,1)=ry;
xe(6,1)=rz;

end