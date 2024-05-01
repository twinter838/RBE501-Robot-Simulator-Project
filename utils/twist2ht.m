function T = twist2ht(S,theta)
    % your code here
    omega=S(1:3);
    omegaBracket=[0,-omega(3),omega(2);
        omega(3),0,-omega(1);
        -omega(2),omega(1),0];
    V=S(4:6);
    % If needed, you can calculate a rotation matrix with:
    R = axisangle2rot(omega,theta);
    P=(eye(3)*theta+(1-cos(theta))*omegaBracket+(theta-sin(theta))*omegaBracket^2)*V;
    T=[R,P;
        0,0,0,1];
end