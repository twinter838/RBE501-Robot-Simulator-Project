function tauList = moveAlongTraj(Robot,q,qOld)
tauList=[];
params.q0=qOld;
params.q1=q;
params.v0=[0,0,0,0,0,0,0]';
params.v1=[0,0,0,0,0,0,0]';
params.t0=0;
params.t1=0.1;
params.dt=0.01;
params.a0=[0,0,0,0,0,0,0]';
params.a1=[0,0,0,0,0,0,0]';

traj=make_trajectory("quintic",params)
for i=1:11

show(Robot,traj.q(i,:)',Visuals="on",Collisions="off",FastUpdate=true,PreservePlot=false)

tauList=[tauList,gravityCompensation(Robot,q)];
drawnow
pause(0.01)
end