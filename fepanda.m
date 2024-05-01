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
    fig = uifigure('Name', 'Franka Emika Panda - 7 DoF Robot', 'Position', [100, 100, 1000, 800]);

    % Define panel positions
    panelLeftPos = [10, 50, 580, 780];
    panelRightPos = [610, 50, 380, 780];

    % Create panels
    robotPanel = uipanel(fig, 'Title', 'Robot - Franka Emika Panda', 'Position', panelLeftPos);
    controlPanel = uipanel(fig, 'Title', 'Control Panel', 'Position', panelRightPos);

    % Axes for robot visualization (main and sub-axes)
    mainAxes = uiaxes(robotPanel, 'Position', [10, 300, 550, 450]);
    subAxes1 = uiaxes(robotPanel, 'Position', [10, 140, 550, 140]);
    subAxes2 = uiaxes(robotPanel, 'Position', [10, 10, 550, 140]);
    subAxes3 = uiaxes(controlPanel, 'Position', [10, 140, 330, 140]);
    subAxes4 = uiaxes(controlPanel, 'Position', [10, 10, 330, 140]);

    % Create the X label and text input
    xLabel = uilabel(controlPanel, 'Text', 'X', 'Position', [30, 720, 50, 25]);
    xEdit = uieditfield(controlPanel, 'numeric', 'Position', [10, 700, 50, 25]);
    
    % Create the Y label and text input
    yLabel = uilabel(controlPanel, 'Text', 'Y', 'Position', [90, 720, 50, 25]);
    yEdit = uieditfield(controlPanel, 'numeric', 'Position', [70, 700, 50, 25]);
    
    % Create the Z label and text input
    zLabel = uilabel(controlPanel, 'Text', 'Z', 'Position', [150, 720, 50, 25]);
    zEdit = uieditfield(controlPanel, 'numeric', 'Position', [130, 700, 50, 25]);

    % Create the R label and text input
    xLabel = uilabel(controlPanel, 'Text', 'Roll', 'Position', [200, 720, 50, 25]);
    xEdit = uieditfield(controlPanel, 'numeric', 'Position', [190, 700, 50, 25]);
    
    % Create the P label and text input
    yLabel = uilabel(controlPanel, 'Text', 'Pitch', 'Position', [260, 720, 50, 25]);
    yEdit = uieditfield(controlPanel, 'numeric', 'Position', [250, 700, 50, 25]);
    
    % Create the Y label and text input
    zLabel = uilabel(controlPanel, 'Text', 'Yaw', 'Position', [320, 720, 50, 25]);
    zEdit = uieditfield(controlPanel, 'numeric', 'Position', [310, 700, 50, 25]);
    
    % Create the Payload label and text input
    payloadLabel = uilabel(controlPanel, 'Text', 'Payload', 'Position', [12, 670, 50, 25]);
    payloadEdit = uieditfield(controlPanel, 'numeric', 'Position', [10, 650, 50, 25]);

    % FK and IK buttons
    fkButton = uibutton(controlPanel, 'Text', 'FK', 'Position', [80, 650, 75, 25]);
    ikButton = uibutton(controlPanel, 'Text', 'IK', 'Position', [180, 650, 75, 25]);

    % Home button
    homeButton = uibutton(controlPanel, 'Text', 'Home', 'Position', [280, 650, 75, 25]);

    % Define properties for each slider
    sliderProperties = {
        {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'},  % Slider labels
        [10, 10, 10, 10, 10, 10, 10],                % X positions for sliders
        [630, 590, 550, 510, 470, 430, 390],          % Y positions for sliders
        [260, 260, 260, 260, 260, 260, 260],          % Widths for sliders
        [-180, 180]                              % Limits for sliders (assuming same for all for simplicity)
    };

    % Create sliders
    for i = 1:length(sliderProperties{1})
        sliderLabel = sliderProperties{1}{i};
        xPosition = sliderProperties{2}(i);
        yPosition = sliderProperties{3}(i);
        width = sliderProperties{4}(i);
        limits = sliderProperties{5};
        
        % Label for slider
        uilabel(controlPanel, 'Text', sliderLabel, ...
                'Position', [xPosition, yPosition - 20, 50, 22]);
        
        % Slider control
        uislider(controlPanel, ...
                 'Position', [xPosition + 60, yPosition, width, 3], ...
                 'Limits', limits);

    end

    % Set callbacks for FK and IK buttons if needed
    % fkButton.ButtonPushedFcn = @(btn,event) executeFK();
    % ikButton.ButtonPushedFcn = @(btn,event) executeIK();

    % Set callback for Home button if needed
    homeButton.ButtonPushedFcn = @(btn,event) goHomePosition();

    % Optionally you can set up more detailed aspects like toolbars, menus, etc.
end




function executeFK()
    disp('Executing Forward Kinematics');
    % Add FK computation code here

end

function executeIK()
    disp('Executing Inverse Kinematics');
    % Add IK computation code here
end

function goHomePosition()
    disp('Resetting to Home Position');
    % Add code to reset the robot to its home configuration here
    q=[0,0,0,0,0,0,0]'
    EEpos=fkinePanda(kinematicModel,q,"space")

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

