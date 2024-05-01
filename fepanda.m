function createFrankaEmikaPandaSimulatorApp()
    % Add the library path if required
    addpath('lib');
    addpath("Panda\meshes\collision\");
    addpath("Panda\meshes\visual\");
    addpath("Panda\tests\");

    global robot kinematicModel;

    % Main figure
    f = figure('Name', 'Franka Emika Panda 7-DOF Robot Simulator', 'NumberTitle', 'off', ...
               'MenuBar', 'none', 'ToolBar', 'none', 'Position', [100, 100, 1024, 768]);

    % UI panels
    robotPanel = uipanel('Parent', f, 'Title', 'Robot - Franka Emika Panda', 'Position', [0.05, 0.2, 0.6, 0.75]);
    plotPanel = uipanel('Parent', f, 'Title', 'Robot Profile', 'Position', [0.7, 0.05, 0.25, 0.9]);
    controlPanel = uipanel('Parent', f, 'Title', 'Control Panel', 'Position', [0.05, 0.05, 0.6, 0.15]);

    % Axes for robot visualization
    robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);

    % Create and plot the robot using the GenerateRobot function
    Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
    %Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
    %Generates a new random config of the robot every time a button is pressed
    % while(true)
    % showdetails(Robot)
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
    tau=gravityCompensation(Robot,q)

    show(robot, 'PreservePlot', false, 'Parent', robotAxes);
    currentQ = zeros(1,7);  % Franka has 7 DOFs

    % Control checkboxes for showing forces
    posXCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FX', 'Position', [10, 10, 50, 25]);
    posYCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FY', 'Position', [70, 10, 50, 25]);
    posZCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FZ', 'Position', [130, 10, 50, 25]);
    
    % Labels and text boxes for robot parameters
    labels = {'X:(m)','Y:(m)', 'Z:(m)', 'Roll:(degrees)', 'Pitch:(degrees)', 'Yaw:(degrees)'};
    for i = 1:length(labels)
        uicontrol('Parent', controlPanel, 'Style', 'text', 'String', labels{i}, ...
                  'Position', [(i-1) * 100,60 , 100, 25]);
    end
    
    % % FK Button
    % FKButton = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'FK',
    %                      'Position', [10, 100, 100, 25], 'Callback', {@FKButtonPushed, Robot, robotAxes});
    % 
    % % IK Button
    % IKButton = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'IK',
    %                      'Position', [120, 100, 100, 25], 'Callback', {@IKButtonPushed, Robot, robotAxes});
    
    % External force label and field
    % externalForceField = uicontrol('Parent', controlPanel, 'Style', 'edit', 'String', '0', ...
    %                                'Position', [300, 10, 100, 25], 'Callback', @externalForceCallback);

    % Button to animate robot
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Home', ...
              'Position', [410, 10, 120, 25], 'Callback', {@animateRobot, robot, robotAxes});
end


function animateRobot(src, event, robot, robotAxes)
    global robot, kinematicModel;
    % Example animation callback
    disp('Animating robot...');
    % Here you would typically update the robot's joint states and re-display it
    currentQ = currentQ + 0.1; % Increment joint angles slightly for demonstration
    show(robot, 'Configuration', currentQ, 'PreservePlot', false, 'Parent', robotAxes);
end

function FKButtonPushed(app, event, robot, robotAxes)
    global robot, kinematicModel
    % Callback for FK button
    % Retrieve joint values from the GUI
    jointAngles = zeros(1,7);
    for i = 1:7
        jointAngles(i) = str2double(app.findobj('Tag', ['Joint' num2str(i)]).Text); % Assuming joint text boxes are tagged 'Joint1', 'Joint2', etc.
    end
    % Calculate end-effector pose using FK
    eePose = fkinePanda (kinematicModel,jointAngles, frame);
    % Update GUI or display results as needed
    disp('End-Effector Position:');
    disp(eePose(1:3)); % Display position part
end

function IKButtonPushed(app, event, robot, robotAxes)
    global robot, kinematicModel;
    % Callback for IK button
    % Retrieve target pose from GUI
    targetPose = zeros(1,7); % Assuming position (x, y, z) and quaternion (w, x, y, z)
    for i = 1:7
        targetPose(i) = str2double(app.findobj('Tag', ['Pose' num2str(i)]).Text); % Assuming pose text boxes are tagged 'Pose1', 'Pose2', etc.
    end
    % Calculate joint configuration using IK
    jointConfig = ikinPanda(targetPose,kinematicModel);
    % Apply the calculated joint configuration to the robot model
    show(robot, 'Configuration', jointConfig, 'PreservePlot', false, 'Parent', robotAxes);
    disp('Calculated Joint Configuration:');
    disp(jointConfig); % Display joint configuration
end

