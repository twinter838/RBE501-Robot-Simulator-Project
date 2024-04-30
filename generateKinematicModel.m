function kinematicModel= generateKinematicModel(Robot)
kinematicModel.M=getTransform(Robot,Robot.homeConfiguration,"world",string(Robot.BodyNames(Robot.NumBodies)));
S=[];
for i=2 :(Robot.NumBodies-1)
currentM=getTransform(Robot,Robot.homeConfiguration,"world",string(Robot.BodyNames(i)));
currentV=currentM(1:3,3);
currentP=currentM(1:3,4);
currentP(3)=0;
currentV
currentP
currentS=[currentV;screw(currentV,currentP)];
S=[S,currentS];
end
kinematicModel.S=S;
end