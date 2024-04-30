Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
%Generates a new random config of the robot every time a button is pressed
%while(true)
q=Robot.homeConfiguration
show(Robot,q,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)
kinematicModel=generateKinematicModel(Robot)
EEpos=fkinePanda(kinematicModel,q,"space")
qTrue=getTransform(Robot,q,"panda_link0","panda_link2")
        currentPose = MatrixLog6(EEpos);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
%qtest=ikinPanda(currentPose,Robot)'
drawnow
tau=gravityCompensation(Robot,q)
waitforbuttonpress
%end


