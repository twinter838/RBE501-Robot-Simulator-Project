function J = jacob0(S,q) 
    % your code here
    J=S(:,1);
    T=eye(4);
    for i=2:size(S,2)
        T=T*twist2ht(S(:,i-1),q(i-1));
        J=[J,adjoint(S(:,i),T)];        
    end
    
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
end