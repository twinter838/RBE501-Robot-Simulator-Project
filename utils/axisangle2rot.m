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