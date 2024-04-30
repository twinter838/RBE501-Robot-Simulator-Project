function kinematicModel= generateKinematicModel(Robot)
kinematicModel.M=getTransform(Robot,Robot.homeConfiguration,"panda_link0",string(Robot.BodyNames(Robot.NumBodies)));
S=[];
for i=2 :(Robot.NumBodies-1)
currentM=getTransform(Robot,Robot.homeConfiguration,"panda_link0",string(Robot.BodyNames(i)));
currentV=currentM(1:3,3);
currentP=currentM(1:3,4);
currentV
currentP
currentS=[currentV;screw(currentV,currentP)];
S=[S,currentS];
end
kinematicModel.S=S;
end