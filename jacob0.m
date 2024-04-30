function J = jacob0(S,q)
    % your code here
    dof = size(S);
    dof = dof(2);
    
    J = zeros(6, dof);

    
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
    
    E = eye(6);
    
    TAccum = eye(4);
        
    for i = 1:dof
        
        %current product of all e mats up to i 
        %E = E * ei
        
        T = twist2ht(S(:,i),q(i));
        TAccum = TAccum * T;
        
        R = TAccum(1:3,1:3);
        p = TAccum(1:3,4);
        px = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
    
        vNew = zeros(6);
        vNew(1:3,1:3) = R;
        vNew(4:6,4:6) = R;
        vNew(4:6,1:3) = px * R;
        
        %Ad mat of the E mat
        A = vNew;
        
        J(:,i) = A * S(:,i);
    end 
end