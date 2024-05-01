function createFrankaEmikaPandaSimulatorApp()
    % Add the library path if required
    addpath('lib');
    addpath("Panda\meshes\collision\");
    addpath("Panda\meshes\visual\");
    addpath("Panda\tests\");

    % global robot kinematicModel;

  
    % Create and plot the robot using the GenerateRobot function
    Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
    createGUI(Robot);
    %Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
    %Generates a new random config of the robot every time a button is pressed
    % while(true)
    % showdetails(Robot)
    q=Robot.randomConfiguration
    %q=[0,pi/2,0,0,0,0,0]'
    % show(Robot,q,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)

    kinematicModel=generateKinematicModel(Robot);
    
    
    % EEpos=fkinePanda(kinematicModel,q,"space")
    % qTrue=getTransform(Robot,q,"panda_link8")
    %    currentPose = MatrixLog6(EEpos);
    %         currentPose = [currentPose(3,2) ...
    %                        currentPose(1,3) ...
    %                        currentPose(2,1) ...
    %                        currentPose(1:3,4)']';
    % qtest=ikinPanda(currentPose,kinematicModel)'
    % drawnow
    % tau=gravityCompensation(Robot,q)
    % 
    % % show(robot, 'PreservePlot', false, 'Parent', robotAxes);
    % currentQ = zeros(1,7);  % Franka has 7 DOFs

  
    
    % % Create text boxes for each parameter
    % textBoxPositions = [0, 40;100, 40; 200, 40; 300, 40; 400, 40; 500, 40]; % Adjust positions as needed
    % for i = 1:numLabels
    %     uicontrol('Parent', controlPanel, 'Style', 'edit', 'String', '0', ...
    %               'Position', [textBoxPositions(i, 1), textBoxPositions(i, 2), 100, 25], ...
    %               'Callback', @(src, event) externalPathCallback(src, event, i));
    % end

end

function createGUI(robot)
    % Create the main figure
    fig = uifigure('Name', 'Franka Emika Panda - 7 DoF Robot', 'Position', [100, 100, 944, 754]);

    % Define panel positions
    panelLeftPos = [10, 50, 574, 674];
    panelRightPos = [594, 50, 340, 674];

    % Create panels
    robotPanel = uipanel(fig, 'Title', 'Robot - Franka Emika Panda', 'Position', panelLeftPos);
    controlPanel = uipanel(fig, 'Title', 'Control Panel', 'Position', panelRightPos);

    % Axes for robot visualization (main and sub-axes)
    mainAxes = uiaxes(robotPanel, 'Position', [10, 270, 554, 384]);
    subAxes1 = uiaxes(robotPanel, 'Position', [10, 140, 174, 120]);
    subAxes2 = uiaxes(robotPanel, 'Position', [200, 140, 174, 120]);
    subAxes3 = uiaxes(robotPanel, 'Position', [390, 140, 174, 120]);

    % Edit fields and sliders for X, Y, Z, and payload
    % Constants for layout
    baseX = 10; % Base x-coordinate for the first control
    spacingX = 160; % Horizontal spacing between controls
    
    % Constants for layout
    baseX = 10; % Starting position for X controls
    spacingX = 170; % Horizontal spacing between controls
    controlWidth = 150; % Width for sliders and edit fields
    labelWidth = 50;  % Width for labels
    
    % Adjusted XYZ control positions
    labelsXYZ = {'X', 'Y', 'Z'};
    for i = 1:length(labelsXYZ)
        % Label
        uilabel(controlPanel, 'Text', labelsXYZ{i}, 'Position', [baseX + (i-1) * spacingX, 180, labelWidth, 22]);
        % Numeric edit field
        uieditfield(controlPanel, 'numeric', 'Position', [baseX + (i-1) * spacingX, 150, controlWidth, 22]);
        % Slider
        uislider(controlPanel, 'Position', [baseX + (i-1) * spacingX, 130, controlWidth, 3], 'Limits', [-10 10]);
    end
    
    % Adjusted RPY control positions
    labelsRPY = {'Pitch', 'Yaw'};
    baseY = 100;  % Starting position for Y controls
    spacingY = 50;  % Vertical spacing between controls
    for i = 1:length(labelsRPY)
        % Label
        uilabel(controlPanel, 'Text', labelsRPY{i}, 'Position', [baseX, baseY - (i-1) * spacingY, labelWidth, 22]);
        % Numeric edit field
        uieditfield(controlPanel, 'numeric', 'Position', [baseX + labelWidth, baseY - (i-1) * spacingY, controlWidth, 22]);
        % Slider
        uislider(controlPanel, 'Position', [baseX, baseY - (i-1) * spacingY - 20, controlWidth + labelWidth, 3], 'Limits', [-180 180]);
    end
    
    % Align FK, IK, and Home buttons
    buttonWidth = 80;
    buttonSpacing = 10;
    % Define the button labels
    buttons = {'FK', 'IK', 'Home'};
    
    % Loop to create each button
    for i = 1:length(buttons)
        uibutton(controlPanel, 'Text', buttons{i}, ...
                 'Position', [baseX + (i-1) * (buttonWidth + buttonSpacing), 50, buttonWidth, 22]);
    end


    % Home button
    % homeButton = uibutton(controlPanel, 'Text', 'Home', 'Position', [230, 80, 100, 22]);

    % Optionally set callbacks for buttons
end



function executeFK()
    disp('Executing Forward Kinematics');
    % Add FK computation code here
end

function executeIK()
    disp('Executing Inverse Kinematics');
    % Add IK computation code here
end

function resetToHomePosition()
    disp('Resetting to Home Position');
    % Add code to reset the robot to its home configuration here
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

