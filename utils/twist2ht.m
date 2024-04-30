function T = twist2ht(S,theta)
    % your code here
    
    % If needed, you can calculate a rotation matrix with:
    % R = axisangle2rot(omega,theta);
    I = [1 0 0; 0 1 0; 0 0 1];
    w = [S(1) S(2) S(3)];
    wx = [0 -S(3) S(2); S(3) 0 -S(1); -S(2) S(1) 0];
    v = [S(4) S(5) S(6)]';
    e = I + sin(theta) .* wx + (1-cos(theta)) .* wx * wx;
    
    d = (I * theta + (1 - cos(theta)) .* wx + (theta - sin(theta)) .* wx * wx) * v;
    
    T = zeros(4);
    
    
    
    T(1, 1) = e(1, 1);
    T(1, 2) = e(1, 2);
    T(1, 3) = e(1, 3);
    
    T(2, 1) = e(2, 1);
    T(2, 2) = e(2, 2);
    T(2, 3) = e(2, 3);
    
    T(3, 1) = e(3, 1);
    T(3, 2) = e(3, 2);
    T(3, 3) = e(3, 3);
    
    T(1, 4) = d(1);
    T(2, 4) = d(2);
    T(3, 4) = d(3);
    
    T(4, 4) = 1; 
    
    
end