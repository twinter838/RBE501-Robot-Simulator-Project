clear, clc, close all
addpath('./Panda');
addpath('./Panda/meshes/');
addpath('./Panda/tests/');
addpath('./Panda/meshes/visual/');
addpath('./Panda/meshes/collision/');

Robot = GenerateRobot();
% %Generates a new random config of the robot every time a button is pressed
% while(true)
%     show(Robot,Robot.randomConfiguration,Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
%     drawnow
%     waitforbuttonpress
% end

function Robot=GenerateRobot(urdf,useVectorFormat,gravity)
    arguments
    urdf='panda.urdf'
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
    
    disp("Showing Robot")
    Robot=importrobot(urdf);
    if(useVectorFormat==true)
    Robot.DataFormat='Column'
    end
    Robot.Gravity=gravity;
end