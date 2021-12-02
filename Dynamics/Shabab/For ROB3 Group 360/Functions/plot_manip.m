function plot_manip(q,base,rot)
%rot is the rotation of the toll at the EE for apofgi singularuity
q1=q(1); %rad
q2=q(2);
q3=q(3);
q4=q(4);


% lengths of robot arm in mm
L1 = 77.8*10^-3; %meter
L2 = 2.2*10^-3;
L3 = 147.69*10^-3;
L4 = 28*10^-3;
L5 = 75.4*10^-3;

Rb=eulertoR(base(4:6));
% base is the position of the base frame wrt world frame

%Transformation

Aw0=[Rb,base(1:3);
    0 0 0  1];


A01 = Aw0*[cos(q1) 0 -sin(q1) L2*cos(q1);
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

A34(1:3,1:3)=A34(1:3,1:3)*rot;
A02 = A01*A12;
A03 = A02*A23;
A0E = A03*A34;

R0E=A0E(1:3,1:3)*rot';



x0 = base(1);
y0 = base(2);
z0 = base(3);

x1= A01(1,4);
y1=A01(2,4);
z1=A01(3,4);

x2= A02(1,4);
y2=A02(2,4);
z2=A02(3,4);

x3= A03(1,4);
y3=A03(2,4);
z3=A03(3,4);


x4= A0E(1,4);
y4=A0E(2,4);
z4=A0E(3,4);

[X, Y, Z] = cylinder2P(0.012,10,[x0 y0 z0],[x1 y1 z1]);
surf(X, Y, Z);
%surf(X, Y, Z,'FaceColor',[0.5 0.5 0.5]);
[X, Y, Z] = cylinder2P(0.012,10,[x1 y1 z1],[x2 y2 z2]);
surf(X, Y, Z);
[X, Y, Z] = cylinder2P(0.012,10,[x2 y2 z2],[x3 y3 z3]);
surf(X, Y, Z);
[X, Y, Z] = cylinder2P(0.012,10,[x3 y3 z3],[x4 y4 z4]);
surf(X, Y, Z);

xer=[x4 y4 z4]'+0.02*R0E(1:3,1);
xel=[x4 y4 z4]'-0.02*R0E(1:3,1);

xul=xel+0.02*R0E(1:3,3);
[X, Y, Z] = cylinder2P(0.003,10,[x4 y4 z4],[xul(1) xul(2) xul(3)]);
surf(X, Y, Z,'FaceColor',[0.5 0.5 0.5]);


xur=xer+0.02*R0E(1:3,3);
[X, Y, Z] = cylinder2P(0.003,10,[x4 y4 z4],[xur(1) xur(2) xur(3)]);
surf(X, Y, Z,'FaceColor',[0.5 0.5 0.5]);




%stick diagram
plot3(x0,y0,z0,'o','MarkerFaceColor','k','MarkerEdgeColor','k');
% plot3([x0,x1],[y0,y1],[z0,z1],'LineWidth',3);
% plot3([x1,x2],[y1,y2],[z1,z2],'LineWidth',3);
% plot3([x2,x3],[y2,y3],[z2,z3],'LineWidth',3);
% plot3([x3,x4],[y3,y4],[z3,z4],'LineWidth',3);
plot3(x4,y4,z4,'o','MarkerFaceColor','k','MarkerEdgeColor','k');

drawframe(A0E, 0.075)
drawframe(Aw0, 0.075)




function [X,Y,Z] = make_circle(r,N1,N2)
r=linspace(0,r,N1);
phi=linspace(-pi,pi,N2);
for i=1:N1
    for j=1:N2
        X(i,j)=r(i)*cos(phi(j));
        Y(i,j)=r(i)*sin(phi(j));
        Z(i,j)=0;
    end
end

function [X,Y,Z] = make_cone(r,l,N1,N2)
x = linspace(0,l,N1);
phi=linspace(0,2*pi,N2);
for i=1:N1
    for j=1:N2
        X(i,j)=x(i);
        Y(i,j)=r*x(i)*cos(phi(j));
        Z(i,j)=r*x(i)*sin(phi(j));
    end
end

function [X,Y,Z] = make_cylinder(r,h,N1,N2)
x=linspace(0,h,N1);
phi=linspace(0,pi,N2);
y=r*cos(phi);
for i=1:N1
    for j=1:N2
        X(i,j)=x(i);
        Y(i,j)=y(j);
        Z(i,j)=sqrt(r^2-y(j)^2);
    end
end

function [X,Y,Z] = make_plane(l1,l2,N1,N2)
x=linspace(0,l1,N1);
y=linspace(0,l2,N2);
for i=1:N1
    for j=1:N2
        X(i,j)=x(i);
        Y(i,j)=y(j);
        Z(i,j)=0;
    end
end

function [X,Y,Z] = make_sphere(r,N1,N2)
theta = linspace(0,pi/2,N1);
phi=linspace(0,2*pi,N2);
for i=1:N1
    for j=1:N2
        X(i,j)=r*sin(theta(i))*cos(phi(j));
        Y(i,j)=r*sin(theta(i))*sin(phi(j));
        Z(i,j)=r*cos(theta(i));
    end
end












