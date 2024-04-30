Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
%Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
%Generates a new random config of the robot every time a button is pressed
while(true)
q=Robot.homeConfiguration
show(Robot,q,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)
kinematicModel=generateKinematicModel(Robot);
%kinematicModel.M=getTransform(Robot,Robot.homeConfiguration,"panda_link0","panda_link2");
%kinematicModel.S=kinematicModel.S(:,1:3);

EEpos=fkinePanda(kinematicModel,q,"space")
qTrue=getTransform(Robot,q,"panda_link8")
%qtest=ikinPanda(currentPose,Robot)'
drawnow
tau=gravityCompensation(Robot,q);
waitforbuttonpress
end


