Robot=GenerateRobot()
%Generates a new random config of the robot every time a button is pressed
while(true)
show(Robot,Robot.randomConfiguration,Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
drawnow
waitforbuttonpress
end