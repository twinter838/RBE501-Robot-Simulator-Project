Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
%Generates a new random config of the robot every time a button is pressed
%while(true)
q=Robot.randomConfiguration
show(Robot,q,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)
kinematicModel=generateKinematicModel(Robot)
%kinematicModel.S=kinematicModel.S(:,1:3)

EEpos=fkinePanda(kinematicModel,q,"space")
qTrue=getTransform(Robot,q,"panda_link0","panda_link8")
%qtest=ikinPanda(currentPose,Robot)'
drawnow
tau=gravityCompensation(Robot,q)
waitforbuttonpress
%end


