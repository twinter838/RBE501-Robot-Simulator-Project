function ik = ikinPanda(targetPose)
    L1 = 0.333;
    L2 = 0.316;
    L3 = 0.088;
    L4 = 0.384;
    L5 = 0.107;

    p1 = [0 0 L1]';
    p2 = [0 0 L1]';
    p3 = [0 0 (L1+L2)]';
    p4 = [L3 0 (L1+L2)]';
    p5 = [0 0 (L1+L2+L4)]';
    p6 = [0 0 (L1+L2+L4)]';
    p7 = [L3 0 (L1+L2+L4)]';
    pF = [L3 0 (L1+L2+L4-L5)]';

    x = [1 0 0];
    y = [0 1 0];
    z = [0 0 1];

    wX = [0 0 0;
          0 0 1;
          0 -1 0];    
    wY = [0 0 -1;
          0 0 0;
          1 0 0];
    wZ = [0 1 0;
          -1 0 0;
          0 0 0];

    S = [0 0 0 0  0 0 0;
        0 0 0 0  0 0 0;
        0 0 0 0  0 0 0;
        0 0 0 0  0 0 0;
        0 0 0 0  0 0 0;
        0 0 0 0  0 0 0];


    v1 = wZ * p1;
    v2 = -wY*p2;
    v3 = wZ*p3;
    v4 = -wY*p4;
    v5 = wZ*p5;
    v6 = -wY*p6;
    v7 = -wZ*p7;


    S(:,1) = [z'; v1];
    S(:,2) = [-y'; v2];
    S(:,3) = [z'; v3];
    S(:,4) = [-y'; v4];
    S(:,5) = [z'; v5];
    S(:,6) = [-y'; v6];
    S(:,7) = [-z'; v7];




    M = [ 1 0 0 L3;
          0 -1 0 0;
          0 0 -1 (L1+L2+L4-L5);
          0 0 0 1];


    
    % 
    % %start of ik stuffs
    % 
    currentQ = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 pi/2]';
    currentT = fkine(S, M, currentQ, "space"); 
    currentPose = MatrixLog6(currentT);
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';


    distance = norm(targetPose - currentPose);


    %_______________________
    i = 0;
 while distance > .00000001 
        S;
        currentQ;
        J = jacob0(S, currentQ);
        diff = targetPose - currentPose;
        dist = norm(diff);
    
        deltaQ = J\diff;
        currentQ = currentQ + deltaQ;

        currentT = fkine(S, M, currentQ, "space");
        currentPose = MatrixLog6(currentT);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

        i = i + 1;
        distance = norm(targetPose - currentPose);

 end 



    %___________

    distance;
    currentQ;
    currentT = fkine(S, M, currentQ, "space");
    currentPose = MatrixLog6(currentT);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    targetPose;


    % need to output q 
    ik = currentQ;
end 
