function G = spatialInertialMatrix (m, r, l)

G = zeros(6,6,length(l));

for i= 1:7

    G(1, 1, i) = m(i) * (3 * r(i)^2 + l(i)^2) / 12;
    G(2, 2, i) = m(i) * (3 * r(i)^2 + l(i)^2) / 12;
    G(3, 3, i) = m(i) * r(i)^2 / 2;
    G(4, 4, i) = m(i);
    G(5, 5, i) = m(i);
    G(6, 6, i) = m(i);
end


end