function createFrankaEmikaPandaSimulatorApp()
    % Add the library path if required
    clc;
    addpath('utils');
    addpath("Panda\meshes\collision\");
    addpath("Panda\meshes\visual\");
    addpath("Panda\tests\");

    global Robot kinematicModel;
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit;
    global  mainAxes posAxes velAxes accAxes torAxes;
    global Joints;
    global fkButton ikButton homeButton ResetButton GcompButton;

  
    % Create and plot the robot using the GenerateRobot function
    Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
    createGUI(Robot);

    % show(Robot, 'PreservePlot', false, 'Parent', mainAxes);
    % show(Robot,Robot.homeConfiguration,Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false);
    Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
    %Generates a new random config of the robot every time a button is pressed
    % while(true)
    % showdetails(Robot)
    % q=Robot.randomConfiguration
    % q=[0,pi/2,0,0,0,0,0]'
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


end

function createGUI(Robot)
   % Declare global variables for GUI components
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit;
    global mainAxes posAxes velAxes accAxes torAxes;
    global fkButton ikButton homeButton ResetButton GcompButton;
    global slider1 slider2 slider3 slider4 slider5 slider6 slider7;
    global valueDisplay1 valueDisplay2 valueDisplay3 valueDisplay4 valueDisplay5 valueDisplay6 valueDisplay7;
    global Joints;

    % Initialize array to store slider values
    Joints = zeros(1, 7);

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
    % robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
    posAxes = uiaxes(robotPanel, 'Position', [10, 140, 550, 140]);
    velAxes = uiaxes(robotPanel, 'Position', [10, 10, 550, 140]);
    accAxes = uiaxes(controlPanel, 'Position', [10, 140, 330, 140]);
    torAxes = uiaxes(controlPanel, 'Position', [10, 10, 330, 140]);

    % Plot the robot in the mainAxes
    % show(Robot, 'PreservePlot', false, 'Parent', mainAxes);
   

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
    rLabel = uilabel(controlPanel, 'Text', 'Roll', 'Position', [200, 720, 50, 25]);
    rEdit = uieditfield(controlPanel, 'numeric', 'Position', [190, 700, 50, 25]);
    
    % Create the P label and text input
    pLabel = uilabel(controlPanel, 'Text', 'Pitch', 'Position', [260, 720, 50, 25]);
    pEdit = uieditfield(controlPanel, 'numeric', 'Position', [250, 700, 50, 25]);
    
    % Create the Y label and text input
    yaLabel = uilabel(controlPanel, 'Text', 'Yaw', 'Position', [320, 720, 50, 25]);
    yaEdit = uieditfield(controlPanel, 'numeric', 'Position', [310, 700, 50, 25]);
    
    % Create the Payload label and text input
    payloadLabel = uilabel(controlPanel, 'Text', 'Payload', 'Position', [12, 670, 50, 25]);
    payloadEdit = uieditfield(controlPanel, 'numeric', 'Position', [10, 650, 50, 25]);

    % FK and IK buttons
    fkButton = uibutton(controlPanel, 'Text', 'FK', 'Position', [80, 650, 75, 25]);
    ikButton = uibutton(controlPanel, 'Text', 'IK', 'Position', [180, 650, 75, 25]);

    % Home button
    homeButton = uibutton(controlPanel, 'Text', 'Home', 'Position', [280, 650, 75, 25]);

    % Reset button
    ResetButton = uibutton(controlPanel, 'Text', 'Reset', 'Position', [250, 300, 75, 30]);

    % Gravity Comp
    GcompButton = uibutton(controlPanel, 'Text', 'Gravity Compensation', 'Position', [30, 300, 150, 30]);




    
    % Inside the function or script
    % Slider 1
    uilabel(controlPanel, 'Text', 'Joint1', 'Position', [10, 630 - 20, 50, 22]);
    slider1 = uislider(controlPanel, 'Position', [10 + 60, 630, 240, 3], 'Limits', [-180, 180]);
    valueDisplay1 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 630 - 20, 40, 22], 'Value', slider1.Value, 'Editable', 'off');
    slider1.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay1, 1, Joints);
    
    % Slider 2
    uilabel(controlPanel, 'Text', 'Joint2', 'Position', [10, 590 - 20, 50, 22]);
    slider2 = uislider(controlPanel, 'Position', [10 + 60, 590, 240, 3], 'Limits', [-180, 180]);
    valueDisplay2 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 590 - 20, 40, 22], 'Value', slider2.Value, 'Editable', 'off');
    slider2.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay2, 2, Joints);
    
    % Slider 3
    uilabel(controlPanel, 'Text', 'Joint3', 'Position', [10, 550 - 20, 50, 22]);
    slider3 = uislider(controlPanel, 'Position', [10 + 60, 550, 240, 3], 'Limits', [-180, 180]);
    valueDisplay3 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 550 - 20, 40, 22], 'Value', slider3.Value, 'Editable', 'off');
    slider3.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay3, 3, Joints);
    
    % Slider 4
    uilabel(controlPanel, 'Text', 'Joint4', 'Position', [10, 510 - 20, 50, 22]);
    slider4 = uislider(controlPanel, 'Position', [10 + 60, 510, 240, 3], 'Limits', [-180, 180]);
    valueDisplay4 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 510 - 20, 40, 22], 'Value', slider4.Value, 'Editable', 'off');
    slider4.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay4, 4, Joints);
    
    % Slider 5
    uilabel(controlPanel, 'Text', 'Joint5', 'Position', [10, 470 - 20, 50, 22]);
    slider5 = uislider(controlPanel, 'Position', [10 + 60, 470, 240, 3], 'Limits', [-180, 180]);
    valueDisplay5 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 470 - 20, 40, 22], 'Value', slider5.Value, 'Editable', 'off');
    slider5.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay5, 5, Joints);
    
    % Slider 6
    uilabel(controlPanel, 'Text', 'Joint6', 'Position', [10, 430 - 20, 50, 22]);
    slider6 = uislider(controlPanel, 'Position', [10 + 60, 430, 240, 3], 'Limits', [-180, 180]);
    valueDisplay6 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 430 - 20, 40, 22], 'Value', slider6.Value, 'Editable', 'off');
    slider6.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay6, 6, Joints);
    
    % Slider 7
    uilabel(controlPanel, 'Text', 'Joint7', 'Position', [10, 390 - 20, 50, 22]);
    slider7 = uislider(controlPanel, 'Position', [10 + 60, 390, 240, 3], 'Limits', [-180, 180]);
    valueDisplay7 = uieditfield(controlPanel, 'numeric', 'Position', [10 + 240 + 80, 390 - 20, 40, 22], 'Value', slider7.Value, 'Editable', 'off');
    slider7.ValueChangingFcn = @(sld,event) updateSliderValueDisplay(sld, event, valueDisplay7, 7, Joints);
    
    % Function to update slider value display and store in array
    function updateSliderValueDisplay(slider, event, valueDisplay, jointNumber, Joints)
        valueDisplay.Value = event.Value;
        Joints(jointNumber) = slider.Value;
    end
    % Set callbacks for FK and IK buttons if needed
    fkButton.ButtonPushedFcn = @(btn, event) executeFK(getCurrentInputs());
    ikButton.ButtonPushedFcn = @(btn, event) executeIK(getCurrentInputs());
    % Reset values of input counters
    ResetButton.ButtonPushedFcn = @(btn,event) executeIK();

    % Set callback for Home button if needed
    homeButton.ButtonPushedFcn = @(btn,event) goHomePosition();

    % Optionally you can set up more detailed aspects like toolbars, menus, etc.
end


% Function to gather current inputs from UI components
function inputs = getCurrentInputs()
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit;

    % Assuming xEdit, yEdit, zEdit, rEdit, pEdit, yaEdit, and payloadEdit are
    % defined in the same script or passed as arguments or available as global.
    inputs = struct( ...
        'x', xEdit.Value, ...
        'y', yEdit.Value, ...
        'z', zEdit.Value, ...
        'roll', rEdit.Value, ...
        'pitch', pEdit.Value, ...
        'yaw', yaEdit.Value, ...
        'payload', payloadEdit.Value ...
    );
end


function executeFK(inputs)
    global Joints kinematicModel;
    disp('Executing Forward Kinematics');
    
    % Display joint angles for debugging
    disp('Joint angles (q):');
    disp(Joints);

    % Assuming you have a function fkinePanda to compute forward kinematics
    % Call the function and pass the joint angles to compute the pose
    pose = fkinePanda(kinematicModel, Joints, "space");

    % Optionally display the results from the fkine function
    disp('FK Results:');
    disp(pose);
end



function executeIK(inputs)
    global kinematicModel;
    disp('Executing Inverse Kinematics');
    
    % Extract pose information from the inputs structure
    x = inputs.x;
    y = inputs.y;
    z = inputs.z;
    roll = inputs.roll;
    pitch = inputs.pitch;
    yaw = inputs.yaw;

    % Convert roll, pitch, yaw to rotation matrix
    R = eul2rotm([roll, pitch, yaw]);

    % Assemble transformation matrix
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = [x; y; z];

    % Convert transformation matrix to 6-dimensional vector
    targetPose = MatrixLog6(T);
    targetPoseVector = [targetPose(3,2); targetPose(1,3); targetPose(2,1); targetPose(1:3,4)];

    % Call ikinPanda function with the targetPoseVector
    q = ikinPanda(targetPoseVector, kinematicModel);
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


function plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc)

    global robotPanel;
    % Plotting the joint positions on the  axesPositions
    posAxes = axes('Parent', robotPanel, 'Position', [0.1, 0.75, 0.8, 0.2]);
    velAxes = axes('Parent', robotPanel, 'Position', [0.1, 0.5, 0.8, 0.2]);
    accAxes = axes('Parent', robotPanel, 'Position', [0.1, 0.25, 0.8, 0.2]);
    torAxes = axes('Parent', robotPanel, 'Position', [0.1, 0.05, 0.8, 0.2]);

    % cla(robotAxes);
    plot(axesPositions, t_acc, jointPos_acc(1,:), 'LineWidth', 2);
    hold(axesPositions, 'on');
    for j = 2:size(jointPos_acc, 1)
        plot(axesPositions, t_acc, jointPos_acc(j,:), 'LineWidth', 2);
    end
    hold( axesPositions, 'off');
    title( axesPositions, 'Joint Positions');
    legend( axesPositions, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel( axesPositions, 'Time [s]'), ylabel( axesPositions, 'Joint Position');
    set( axesPositions, 'FontSize', 14);

    % Plotting the joint velocities on the  axesVelocities
    plot(axesVelocities, t_acc, jointVel_acc(1,:), 'LineWidth', 2);
    hold(axesVelocities, 'on');
    for j = 2:size(jointVel_acc, 1)
        plot(axesVelocities, t_acc, jointVel_acc(j,:), 'LineWidth', 2);
    end
    hold( axesVelocities, 'off');
    title( axesVelocities, 'Joint Velocities');
    legend( axesVelocities, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel(axesVelocities, 'Time [s]'), ylabel( axesVelocities, 'Joint Velocities ');
    set(axesVelocities, 'FontSize', 14);


    % Plotting the joint accelerations on the  axesAccelerations
    plot(axesAccelerations, t_acc, jointAcl_acc(1,:), 'LineWidth', 2);
    hold(axesAccelerations, 'on');
    for j = 2:size(jointAcl_acc, 1)
        plot(axesAccelerations, t_acc, jointAcl_acc(j,:), 'LineWidth', 2);
    end
    hold(axesAccelerations, 'off');
    title(axesAccelerations, 'Joint Accelerations');
    legend(axesAccelerations, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel( axesAccelerations, 'Time [s]'), ylabel(axesAccelerations, 'Joint Accelerations');
    set(axesAccelerations, 'FontSize', 14);

    % Plotting the torque profiles on the axesTorques
    plot(axesTorques, t_acc, tau_acc(1,:), 'LineWidth', 2);
    hold(axesTorques, 'on');
    for j = 2:size(tau_acc, 1)
        plot(axesTorques, t_acc, tau_acc(j,:), 'LineWidth', 2);
    end
    hold(axesTorques, 'off');
    title(axesTorques, 'Joint Torques');
    legend(axesTorques, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
    xlabel(axesTorques, 'Time [s]'), ylabel(axesTorques, 'Torque [Nm]');
    set(axesTorques, 'FontSize', 14);

end
