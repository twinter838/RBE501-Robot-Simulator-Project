Robot=GenerateRobot("panda.urdf",true,[0,0,-9.81]);
%Robot=loadrobot("frankaEmikaPanda","DataFormat","column")
%Generates a new random config of the robot every time a button is pressed
show(Robot,Robot.homeConfiguration,Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false);
qOld=Robot.homeConfiguration';
% drawnow
while(true)
% showdetails(Robot)
% q=Robot.randomConfiguration
q=zeros(7,1)
%q=[0,pi/2,0,0,0,0,0]'
show(Robot,q,Visuals="on",Collisions="off",FastUpdate=true,PreservePlot=false)
kinematicModel=generateKinematicModel(Robot);


EEpos=fkinePanda(kinematicModel,q,"space")
qTrue=getTransform(Robot,q,"panda_link8")
   currentPose = MatrixLog6(EEpos);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
%qtest=ikinPanda(currentPose,kinematicModel)'

% tauList=moveAlongTraj(Robot,q,qOld)
    params.q0=qOld';
    params.q1=q;
    params.v0=[0,0,0,0,0,0,0]';
    params.v1=[0,0,0,0,0,0,0]';
    params.t0=0;
    params.t1=0.1;
    params.dt=0.01;
    params.a0=[0,0,0,0,0,0,0]';
    params.a1=[0,0,0,0,0,0,0]';

traj=make_trajectory("quintic",params)
for i = 1:length(traj.q)
    q = traj.q(i,:);
    show(Robot,q',Visuals="on",Collisions="off",FastUpdate=true,PreservePlot=false)
    drawnow
    pause(0.01)
end
% qOld = traj.q(i,:)
%tau=gravityCompensation(Robot,q);
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
% for i=1:11
% 
% show(Robot,traj.q(i,:)',Visuals="off",Collisions="off",FastUpdate=true,PreservePlot=false)
% 
% tau=gravityCompensation(Robot,q);
% drawnow
% pause(0.01)
% end
qOld=q

waitforbuttonpress
end


