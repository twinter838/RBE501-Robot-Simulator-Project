Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
%Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
%Generates a new random config of the robot every time a button is pressed
while(true)
showdetails(Robot)
q=Robot.randomConfiguration
%q=[0,pi/2,0,0,0,0,0]'
show(Robot,q,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)
kinematicModel=generateKinematicModel(Robot);


EEpos=fkinePanda(kinematicModel,q,"space")
qTrue=getTransform(Robot,q,"panda_link8")
   currentPose = MatrixLog6(EEpos);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
qtest=ikinPanda(currentPose,kinematicModel)'
drawnow
tau=gravityCompensation(Robot,q);
waitforbuttonpress
end


