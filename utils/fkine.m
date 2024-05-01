function T = fkine(S,M,q,frame)
    % your code here
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i))
        
            
    T0 = eye(4);
    for i = 1:size(q)
        T0 = T0 * twist2ht(S(:,i), q(i));
    end 
    
        
    if frame == "space"
        T = T0 * M;
    else if frame == "body"
        T = M * T0;    
    end 
end
