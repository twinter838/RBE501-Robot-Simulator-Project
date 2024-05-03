function createFrankaEmikaPandaSimulatorApp()
    % Add the library path if required
    clc;
    addpath('utils');
    addpath('Panda\');
    addpath("Panda\meshes\collision\");
    addpath("Panda\meshes\visual\");
    addpath("Panda\tests\");

    global Robot kinematicModel;
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit;
    global  mainAxes posAxes velAxes accAxes torAxes;
    global Joints;
    global fkButton ikButton homeButton ResetButton GcompButton;
    global RNEParams
    global qOld
    % Create and plot the robot using the GenerateRobot function
    Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
    createGUI();
    kinematicModel=generateKinematicModel(Robot);
    RNEParams.G=spatialInertialMatrix(Robot);
    RNEParams.g=Robot.Gravity;
    RNEParams.M=kinematicModel.Mlist;
    RNEParams.S=kinematicModel.S;
    RNEParams.Ftip=[0,0,0,0,0,0]';
end

function createGUI()
   % Declare global variables for GUI components
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit;
    global mainAxes posAxes velAxes accAxes torAxes;
    global fkButton ikButton homeButton ResetButton GcompButton;
    global slider1 slider2 slider3 slider4 slider5 slider6 slider7;
    global valueDisplay1 valueDisplay2 valueDisplay3 valueDisplay4 valueDisplay5 valueDisplay6 valueDisplay7;
    global Joints Robot;

    % Initialize array to store slider values
    Joints = zeros(1, 7);

    % Create the main figure
    fig = uifigure('Name', 'Franka Emika Panda - 7 DoF Robot', 'Position', [100, 100, 1000, 600]);

    % Define panel positions
    panelLeftPos = [10, 50, 580, 580];
    panelRightPos = [610, 50, 380, 580];

    % Create panels
    robotPanel = uipanel(fig, 'Title', 'Robot - Franka Emika Panda', 'Position', panelLeftPos);
    controlPanel = uipanel(fig, 'Title', 'Control Panel', 'Position', panelRightPos);

    % Axes for robot visualization (main and sub-axes)
    mainAxes = uiaxes(robotPanel, 'Position', [10, 30, 550, 450]);

    qOld=Robot.homeConfiguration;
    % drawnow

    % Call the show() function to generate the plot
    robotPlot = show(Robot, Robot.homeConfiguration, 'Visuals', 'on', 'Collisions', 'off', 'PreservePlot', true);
    
    robotChildren = robotPlot.Children;
    
    % Set the Parent property of the children to mainAxes to plot in the UIAxes
    for i = 1:numel(robotChildren)
        set(robotChildren(i), 'Parent', mainAxes);
    end
    
    % % Adjust the axis limits
    % axis(mainAxes, 'tight');
    % axis(mainAxes, 'equal');
    % 
    % Define custom axis limits
    xlim(mainAxes, [-1.5, 1.5]);
    ylim(mainAxes, [-1.5, 1.5]);
    zlim(mainAxes, [-1.5, 1.5]);
    % Set the desired view angle (e.g., [azimuth, elevation])
    view(mainAxes, [0,0]);

    % show(Robot,Robot.homeConfiguration,Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
    % posLabel = uilabel(controlPanel, 'Text', 'End Effector Position', 'Position', [30, 510, 150, 25]);  %30, 720 
    % posAxes = uiaxes(controlPanel, 'Position', [10, 350, 350, 140]);
    velAxes = uiaxes(controlPanel, 'Position', [10, 270, 350, 120]);
    accAxes = uiaxes(controlPanel, 'Position', [10, 140, 350, 120]);
    torAxes = uiaxes(controlPanel, 'Position', [10, 20, 350, 100]);
   

    % Create the X label and text input
    xLabel = uilabel(controlPanel, 'Text', 'X', 'Position', [30, 520, 50, 25]);  %30, 720 
    xEdit = uieditfield(controlPanel, 'numeric', 'Position', [10, 500, 50, 25]); %10, 700
    
    % Create the Y label and text input
    yLabel = uilabel(controlPanel, 'Text', 'Y', 'Position', [90, 520, 50, 25]);
    yEdit = uieditfield(controlPanel, 'numeric', 'Position', [70, 500, 50, 25]);
    
    % Create the Z label and text input
    zLabel = uilabel(controlPanel, 'Text', 'Z', 'Position', [150, 520, 50, 25]);
    zEdit = uieditfield(controlPanel, 'numeric', 'Position', [130, 500, 50, 25]);

    % Create the R label and text input
    rLabel = uilabel(controlPanel, 'Text', 'Roll', 'Position', [200, 520, 50, 25]);
    rEdit = uieditfield(controlPanel, 'numeric', 'Position', [190, 500, 50, 25]);
    
    % Create the P label and text input
    pLabel = uilabel(controlPanel, 'Text', 'Pitch', 'Position', [260, 520, 50, 25]);
    pEdit = uieditfield(controlPanel, 'numeric', 'Position', [250, 500, 50, 25]);
    
    % Create the Y label and text input
    yaLabel = uilabel(controlPanel, 'Text', 'Yaw', 'Position', [320, 520, 50, 25]);
    yaEdit = uieditfield(controlPanel, 'numeric', 'Position', [310, 500, 50, 25]);
    
    % Create the Payload label and text input
    payloadLabel = uilabel(controlPanel, 'Text', 'Payload', 'Position', [12, 470, 50, 25]);
    payloadEdit = uieditfield(controlPanel, 'numeric', 'Position', [10, 450, 50, 25]);

    % FK and IK buttons
    % fkButton = uibutton(controlPanel, 'Text', 'FK', 'Position', [80, 130, 75, 25]);
    ikButton = uibutton(controlPanel, 'Text', 'Move Robot!', 'Position', [120, 450, 140, 40]);

    % Home button
    homeButton = uibutton(controlPanel, 'Text', 'Home', 'Position', [220, 400, 120, 25]);

    % Reset button
    % ResetButton = uibutton(controlPanel, 'Text', 'Reset', 'Position', [250, 300, 75, 30]);

    % Gravity Comp
    GcompButton = uibutton(controlPanel, 'Text', 'Gravity Comp', 'Position', [20, 400, 120, 25]);
    
  

    % Set callbacks for FK and IK buttons if needed
    % fkButton.ButtonPushedFcn = @(btn, event) executeFK();
    ikButton.ButtonPushedFcn = @(btn, event) executeIK(getCurrentInputs());
    % Reset values of input counters
    % ResetButton.ButtonPushedFcn = @(btn,event) executeIK();
    GcompButton.ButtonPushedFcn = @(btn,event) gravitycomp();
    % Set callback for Home button if needed
    homeButton.ButtonPushedFcn = @(btn,event) goHomePosition();

    % Optionally you can set up more detailed aspects like toolbars, menus, etc.
end


% Function to gather current inputs from UI components
function inputs = getCurrentInputs()
    global xEdit yEdit zEdit rEdit pEdit yaEdit payloadEdit RNEParams;

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
    RNEParams.Ftip=[0,0,0,0,0,-payloadEdit.Value];
end

function gravitycomp()
 global Joints kinematicModel Robot RNEParams qOld;
    qOld=Robot.homeConfiguration;
    q=Robot.randomConfiguration
    params.q0=qOld;
    params.q1=q;
    params.v0=[0,0,0,0,0,0,0]';
    params.v1=[0,0,0,0,0,0,0]';
    params.t0=0;
    params.t1=0.1;
    params.dt=0.01;
    params.a0=[0,0,0,0,0,0,0]';
    params.a1=[0,0,0,0,0,0,0]';
    

    RNEParams.G=spatialInertialMatrix(Robot);
    RNEParams.g=Robot.Gravity;
    RNEParams.M=kinematicModel.Mlist;
    RNEParams.S=kinematicModel.S;

    tauList=[]
    traj=make_trajectory("quintic",params)
    for i = 1:length(traj.q)
        q = traj.q(i,:)
        RNEParams.jointPos=traj.q(i,:);
        RNEParams.jointVel=traj.v(i,:);
        RNEParams.jointAcc=traj.a(i,:);
        tau=rne(RNEParams);
        %tau=tau+gravityCompensation(Robot,q);
        tauList=[tauList,tau];
        drawnow
        pause(0.01)
    end
    tauList
    qOld=q'
    % for i=1:11
    % 
    % show(Robot,traj.q(i,:)',Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
    % 
    % tau=gravityCompensation(Robot,q);
    % drawnow
    % pause(0.01)
    % end
end

function executeFK()
    global Joints kinematicModel Robot;
    disp('Executing Forward Kinematics');

    global kinematicModel Robot robotPanel mainAxes;

    global slider1 slider2 slider3 slider4 slider5 slider6 slider7;
    
    % Extract pose information from the inputs structure
    % x = inputs.x;
    % y = inputs.y;
    % z = inputs.z;
    % roll = inputs.roll;
    % pitch = inputs.pitch;
    % yaw = inputs.yaw;
    % 
    % % Convert roll, pitch, yaw to rotation matrix
    % R = eul2rotm([roll, pitch, yaw]);
    % 
    % % Assemble transformation matrix
    % T = eye(4);
    % T(1:3, 1:3) = R;
    % T(1:3, 4) = [x; y; z];
    % 
    % % Convert transformation matrix to 6-dimensional vector
    % targetPose = MatrixLog6(T);
    % targetPoseVector = [targetPose(3,2); targetPose(1,3); targetPose(2,1); targetPose(1:3,4)];
    % 
    % % Call ikinPanda function with the targetPoseVector
    % q = ikinPanda(targetPoseVector, kinematicModel);
    % qOld=Robot.homeConfiguration;
    % 
    % params.q0=qOld;
    % params.q1=q;
    % params.v0=[0,0,0,0,0,0,0]';
    % params.v1=[0,0,0,0,0,0,0]';
    % params.t0=0;
    % params.t1=0.1;
    % params.dt=0.01;
    % params.a0=[0,0,0,0,0,0,0]';
    % params.a1=[0,0,0,0,0,0,0]';
    % 
    % traj=make_trajectory("quintic",params)
    % for i = 1:length(traj.q)
    %     q = traj.q(i,:);
    %     cla(mainAxes);
    %     % show(Robot,q',Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
    %     robotPlot = show(Robot, q', 'Visuals', 'on', 'Collisions', 'on', 'PreservePlot', false);
    % 
    %     % Get the children (i.e., graphics objects) of the generated plot
    %     robotChildren = robotPlot.Children;
    % 
    %     % Set the Parent property of the children to mainAxes to plot in the UIAxes
    %     for i = 1:numel(robotChildren)
    %         set(robotChildren(i), 'Parent', mainAxes);
    %     end
    % 
    %     % % Adjust the axis limits
    %     % axis(mainAxes, 'tight');
    %     % axis(mainAxes, 'equal');
    %     % 
    %     % Define custom axis limits
    %     xlim(mainAxes, [-1.5, 1.5]);
    %     ylim(mainAxes, [-1.5, 1.5]);
    %     zlim(mainAxes, [-1.5, 1.5]);
    %     % Set the desired view angle (e.g., [azimuth, elevation])
    %     view(mainAxes, [0,0]);
    % 
    % 
    %     drawnow
    %     pause(0.5)
    % end

end



function executeIK(inputs)
    global kinematicModel Robot robotPanel mainAxes RNEParams qOld;
    global velAxes accAxes torAxes;
    disp('Executing Inverse Kinematics');
    
    % Extract pose information from the inputs structure
    x = inputs.x;
    y = inputs.y;
    z = inputs.z;
    roll = inputs.roll;
    pitch = inputs.pitch;
    yaw = inputs.yaw;
    payload = inputs.payload;

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
    params.q0=qOld;
    params.q1=q;
    params.v0=[0,0,0,0,0,0,0]';
    params.v1=[0,0,0,0,0,0,0]';
    params.t0=0;
    params.t1=0.1;
    params.dt=0.01;
    params.a0=[0,0,0,0,0,0,0]';
    params.a1=[0,0,0,0,0,0,0]';
    traj=make_trajectory("quintic",params);
    
    tauList = [];
    velList = [];
    accList = [];
    
    for i = 1:length(traj.q)
        q = traj.q(i,:);
        cla(mainAxes);
        % show(Robot,q',Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
        robotPlot = show(Robot, q', 'Visuals', 'on', 'Collisions', 'off', 'PreservePlot', false);
        RNEParams.jointPos = traj.q(i,:);
        RNEParams.jointVel = traj.v(i,:);
        RNEParams.jointAcc = traj.a(i,:);
        tau = rne(RNEParams);
        
        % Collect joint velocities, accelerations, and torques
        velList = [velList; traj.v(i,:)];
        accList = [accList; traj.a(i,:)];
        tauList = [tauList; tau];
        
        % Get the children (i.e., graphics objects) of the generated plot
        robotChildren = robotPlot.Children;
        
        % Set the Parent property of the children to mainAxes to plot in the UIAxes
        for j = 1:numel(robotChildren)
            set(robotChildren(j), 'Parent', mainAxes);
        end
        
        % Define custom axis limits
        xlim(mainAxes, [-1.5, 1.5]);
        ylim(mainAxes, [-1.5, 1.5]);
        zlim(mainAxes, [-1.5, 1.5]);
        % Set the desired view angle (e.g., [azimuth, elevation])
        view(mainAxes, [0,0]);

        drawnow
        pause(0.1)
    end
    qOld=q';
    % Plot joint velocities on velAxes
    for i = 1:7
        plot(velAxes, velList(:,i));
        hold(velAxes, 'on');
    end
    title(velAxes, 'Joint Velocities');
    xlabel(velAxes, 'Time');
    ylabel(velAxes, 'Velocity');
    hold(velAxes, 'off');
    
    % Plot joint accelerations on accAxes
    for i = 1:7
        plot(accAxes, accList(:,i));
        hold(accAxes, 'on');
    end
    title(accAxes, 'Joint Accelerations');
    xlabel(accAxes, 'Time');
    ylabel(accAxes, 'Acceleration');
    hold(accAxes, 'off');
    
    % Plot joint torques on torAxes
    % Reshape tauList into a 7xT matrix
    tauList = reshape(tauList, [], 7);
    
    % Plot joint torques on torAxes
    for i = 1:7
        plot(torAxes, tauList(:,i));
        hold(torAxes, 'on');
    end
    title(torAxes, 'Joint Torques');
    xlabel(torAxes, 'Time');
    ylabel(torAxes, 'Torque');
    hold(torAxes, 'off');

    
end



function goHomePosition()
     global kinematicModel Robot robotPanel mainAxes RNEParams qOld;
    disp('Resetting to Home Position');
    % 
    % % Extract pose information from the inputs structure
    % x = 0;
    % y = 0;
    % z = 0;
    % roll = 0;
    % pitch = 0;
    % yaw = 0;
    % 
    % % Convert roll, pitch, yaw to rotation matrix
    % R = eul2rotm([roll, pitch, yaw]);
    % 
    % % Assemble transformation matrix
    % T = eye(4);
    % T(1:3, 1:3) = R;
    % T(1:3, 4) = [x; y; z];
    % 
    % % Convert transformation matrix to 6-dimensional vector
    % targetPose = MatrixLog6(T);
    % targetPoseVector = [targetPose(3,2); targetPose(1,3); targetPose(2,1); targetPose(1:3,4)];
    % 
    % % Call ikinPanda function with the targetPoseVector
    % q = ikinPanda(targetPoseVector, kinematicModel);
    % 
    params.q0=qOld;
    params.q1=Robot.homeConfiguration;
    params.v0=[0,0,0,0,0,0,0]';
    params.v1=[0,0,0,0,0,0,0]';
    params.t0=0;
    params.t1=0.1;
    params.dt=0.01;
    params.a0=[0,0,0,0,0,0,0]';
    params.a1=[0,0,0,0,0,0,0]';
    tauList=[]
    % Robot = Robot.homeConfiguration;
    
    traj=make_trajectory("quintic",params)
    for i = 1:length(traj.q)
        q = traj.q(i,:);
        cla(mainAxes);
        % show(Robot,q',Visuals="on",Collisions="on",FastUpdate=true,PreservePlot=false)
        robotPlot = show(Robot, q', 'Visuals', 'on', 'Collisions', 'off', 'PreservePlot', false);
        
        % Get the children (i.e., graphics objects) of the generated plot
        robotChildren = robotPlot.Children;
        RNEParams.jointPos=traj.q(i,:);
        RNEParams.jointVel=traj.v(i,:);
        RNEParams.jointAcc=traj.a(i,:);
        tau=rne(RNEParams);
        %tau=tau+gravityCompensation(Robot,q);
        tauList=[tauList,tau];
        % Set the Parent property of the children to mainAxes to plot in the UIAxes
        for i = 1:numel(robotChildren)
            set(robotChildren(i), 'Parent', mainAxes);
        end
        
        % % Adjust the axis limits
        % axis(mainAxes, 'tight');
        % axis(mainAxes, 'equal');
        % 
        % Define custom axis limits
        xlim(mainAxes, [-1.5, 1.5]);
        ylim(mainAxes, [-1.5, 1.5]);
        zlim(mainAxes, [-1.5, 1.5]);
        % Set the desired view angle (e.g., [azimuth, elevation])
        view(mainAxes, [0,0]);

        
        drawnow
        pause(0.5)
    end
    qOld=Robot.homeConfiguration;
end

function animateRobot(src, event, robot, robotAxes)
    global robot, kinematicModel;
    % Example animation callback
    disp('Animating robot...');
    % Here you would typically update the robot's joint states and re-display it
    currentQ = currentQ + 0.1; % Increment joint angles slightly for demonstration
    show(robot, 'Configuration', currentQ, 'PreservePlot', false, 'Parent', robotAxes);
end

