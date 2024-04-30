Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81])
%Generates a new random config of the robot every time a button is pressed
while(true)
q=Robot.randomConfiguration
show(Robot,q,Visuals="on",Collisions="off",FastUpdate=true,PreservePlot=false)
drawnow
tau=gravityCompensation(Robot,q)
waitforbuttonpress
end


