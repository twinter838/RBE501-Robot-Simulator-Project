function ik = ikinPanda(targetPose,kinematicModel)
S=kinematicModel.S;
M=kinematicModel.M;
   
    % 
    % %start of ik stuffs
    % 
    currentQ = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]';
    currentT = fkine(S, M, currentQ, "space");

    currentPose = MatrixLog6(currentT);
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';


    distance = norm(targetPose - currentPose);
    lambda=1.2;

    %_______________________
    i = 0;
 while distance > .01 && i<50000
        currentQ;
        J = jacob0(S, currentQ);
        deltaQ=J'*pinv(J*J'+lambda^2*eye(6))*(targetPose - currentPose);

        diff = targetPose - currentPose;
        dist = norm(diff);
        dist;
        currentQ = currentQ + deltaQ;

    currentT = fkine(S, M, currentQ, "space");

        currentPose = MatrixLog6(currentT);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        i = i + 1;
        distance = norm(targetPose - currentPose)

 end 



    %___________

    % distance
    currentQ;
    currentT = fkinePanda(kinematicModel,currentQ, "space");
    currentPose = MatrixLog6(currentT);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    targetPose;


    % need to output q 
    ik = currentQ;
end 
