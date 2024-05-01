function AdT = Adjoint(T)


    P = T(1:3, 4);
    R = T(1:3, 1:3);
    AdT = [  R           zeros(3);
            skew(P)*R    R   ];
   
end