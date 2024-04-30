function pose = fkinePanda (q, frame)
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

    S = [0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0];


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
    %S(:,8) = [-z'; -wZ*pF]




    M = [ 1 0 0 L3;
          0 -1 0 0;
          0 0 -1 (L1+L2+L4-L5);
          0 0 0 1];

    %end effector translation on z axis

   % Tee = [1 0 0 0;
   %        0 1 0 0;
   %        0 0 1 L5;
   %        0 0 0 1];




    pose = fkine(S,M,q, frame);

end 
