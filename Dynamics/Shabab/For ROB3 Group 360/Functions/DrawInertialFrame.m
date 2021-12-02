function DrawInertialFrame()

Inertialpoint=[0 0 0]';
line([Inertialpoint(1,1) Inertialpoint(1,1)],[Inertialpoint(2,1) Inertialpoint(2,1)],[Inertialpoint(3,1)+0.5 Inertialpoint(3,1)-0.5 ],'Color','b','LineWidth',2.0)
text(Inertialpoint(1,1),Inertialpoint(2,1),Inertialpoint(3,1)+0.55, 'Z','FontSize',15, 'FontWeight', 'bold', 'Rotation',+15)%,'FontSize',20
line([Inertialpoint(1,1)-0.5 Inertialpoint(1,1)+0.5],[Inertialpoint(2,1) Inertialpoint(2,1)],[Inertialpoint(3,1) Inertialpoint(3,1) ],'Color','b','LineWidth',2.0)
text(Inertialpoint(1,1),Inertialpoint(2,1)+0.55,Inertialpoint(3,1), 'Y','FontSize',15, 'FontWeight', 'bold', 'Rotation',+15)%,'FontSize',20
line([Inertialpoint(1,1) Inertialpoint(1,1)],[Inertialpoint(2,1)-0.5 Inertialpoint(2,1)+0.5],[Inertialpoint(3,1) Inertialpoint(3,1) ],'Color','b','LineWidth',2.0)
text(Inertialpoint(1,1)+0.55,Inertialpoint(2,1),Inertialpoint(3,1), 'X','FontSize',15, 'FontWeight', 'bold', 'Rotation',+15)%,'FontSize',20

end