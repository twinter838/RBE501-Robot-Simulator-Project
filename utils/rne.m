function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
    n = width(params.S);
    S=params.S;
    G=params.G;
    q = params.jointPos;
    qd = params.jointVel;
    qdd = params.jointAcc;
    Ftip = params.Ftip;
    g=params.g
    M=params.M
    V = zeros(6, n+1);
    Vdot = zeros(6, n+1);
    Vdot(:,1) = transpose([0 0 0 -g(1) -g(2) -g(3)]);
    T = zeros(4,4,n+1);
    A = zeros(6,n);
for ii = 1 : n
        %Home Config
        Mi = eye(4);
        for jj = 1 : ii
            Mi = Mi * M(:,:,jj);
        end
        %Transform from space to local frame
        Ai = Adjoint(pinv(Mi)) * S(:,ii);
        A(:,ii) = Ai;
        Ti = twist2ht(Ai, -q(ii)) * pinv(M(:,:,ii));
        T(:,:,ii) = Ti;

        V(:,ii+1) = Ai * qd(ii) + Adjoint(Ti) * V(:,ii);
        Vdot(:,ii+1) = Ai * qdd(ii) + ad(V(:,ii+1)) * Ai * qd(ii) + Adjoint(Ti) * Vdot(:,ii);
    end

    T(:,:,end) = pinv(M(:,:,end));
    F = zeros(6,n+1);
    F(:,end) = Ftip;
    tau = zeros(n,1);
    
    
% Backward iterations
    for ii = n:-1:1
    
        Ti1 = T(:,:,ii+1);
        F(:,ii) = G(:,:,ii) * Vdot(:,ii+1) - ...
                    ad(V(:,ii+1))' * (G(:,:,ii) * V(:,ii+1)) + ...
                    Adjoint(Ti1)' * F(:,ii+1);
        tau(ii) = F(:,ii)' * A(:,ii);
    end

end
function T = twist2ht(S,theta)
    % your code here
    omega=S(1:3);
    omegaBracket=[0,-omega(3),omega(2);
        omega(3),0,-omega(1);
        -omega(2),omega(1),0];
    V=S(4:6);
    % If needed, you can calculate a rotation matrix with:
    R = axisangle2rot(omega,theta);
    P=(eye(3)*theta+(1-cos(theta))*omegaBracket+(theta-sin(theta))*omegaBracket^2)*V
    T=[R,P;
        0,0,0,1]
end
function R = axisangle2rot(omega,theta)
kx = omega(1) ; 
ky = omega(2) ; 
kz = omega(3) ;
C_theta = cos(theta);
S_theta = sin(theta);
V_theta = 1 - C_theta ;

R = [ kx * kx * V_theta + C_theta,       kx * ky * V_theta - kz * S_theta, kx * kz * V_theta + ky * S_theta ; 
       kx * ky * V_theta + kz * S_theta,  ky * ky * V_theta + C_theta,      ky * kz * V_theta - kx * S_theta  ;
       kx * kz * V_theta - ky * S_theta,  kz * ky * V_theta + kx * S_theta, kz * kz * V_theta + C_theta ] ;  
  

end