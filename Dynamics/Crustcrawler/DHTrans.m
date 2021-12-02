
function T = DHTrans(a,alpha,d, thetaOff, theta)
T = eye(4);
for i = 1:1:length(a)
    T = T*iTrans(a(i),alpha(i),d(i), thetaOff(i), theta(i));    
end

end 

function Ti = iTrans(a,alpha, d, thetaOff, theta)
    Ti = [ cos(thetaOff+theta) -sin(thetaOff+theta)                           0              a;
        sin(thetaOff+theta)*cos(alpha) cos(thetaOff+theta)*cos(alpha)          -sin(alpha) -sin(alpha)*d;
        sin(thetaOff+theta)*sin(alpha) cos(thetaOff+theta)*sin(thetaOff+theta) cos(alpha)   cos(alpha)*d;
        0 0 0 1 ];
end