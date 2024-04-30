function S = skew(v)
%skew returns a 3x3 skew symmetric matrix for the given 3x1 vector
%v 3x1 vector
S = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];

end