function kinematicModel= generateKinematicModel(Robot)
kinematicModel.M=round(getTransform(Robot,Robot.homeConfiguration,string(Robot.BodyNames(Robot.NumBodies))),10);
S=double.empty(6,0);
for i=2 :(Robot.NumBodies-1)
currentM=getTransform(Robot,Robot.homeConfiguration,string(Robot.BodyNames(i)))
currentV=currentM(1:3,3);
currentP=currentM(1:3,4);
currentS=[currentV;screw(currentV,currentP)];
S=[S,currentS]
string(Robot.BodyNames(i))
end
S=round(S,10)
kinematicModel.S=S;
end