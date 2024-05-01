function kinematicModel= generateKinematicModel(Robot)

%M Matrix Generation
kinematicModel.M=round(getTransform(Robot,Robot.homeConfiguration,string(Robot.BodyNames(Robot.NumBodies))),10);
S=double.empty(6,0);

%Screw Axis Generation
for i=2 :(Robot.NumBodies-1)
currentM=getTransform(Robot,Robot.homeConfiguration,string(Robot.BodyNames(i)));
currentV=currentM(1:3,3);
currentP=currentM(1:3,4);
currentS=[currentV;screw(currentV,currentP)];
S=[S,currentS];
string(Robot.BodyNames(i))
end
S=round(S,10);
kinematicModel.S=S;

%MList Generation
for i=1 :(Robot.NumBodies-1)
Mlist(:,:,i)=getTransform(Robot,Robot.homeConfiguration,string(Robot.BodyNames(i)),string(Robot.BodyNames(i+1)));
end
kinematicModel.Mlist=Mlist;


end