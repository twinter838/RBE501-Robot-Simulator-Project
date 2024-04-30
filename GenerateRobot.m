function Robot=GenerateRobot(urdf,useVectorFormat,gravity)
arguments
urdf="frankaEmikaPanda.urdf"
useVectorFormat=false;
gravity=[0,0,0]
end
% GenerateRobot Generates a Rigid Body Tree for the specified URDF file
%
% urdf: The filename of the URDF file to load
%
% useVectorFormat: Specifies if the robot should take rigid body
% configurations or column vectors to specifiy its configuration
%
% gravity: Sets the Gravity
%
%

disp("Showing Robot")
Robot=importrobot(urdf,"urdf");
if(useVectorFormat==true)
Robot.DataFormat='Column';
end
Robot.Gravity=gravity;
%Robot.removeBody("panda_leftfinger")
%Robot.removeBody("panda_rightfinger")
end