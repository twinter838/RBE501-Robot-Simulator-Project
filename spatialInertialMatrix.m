function G = spatialInertialMatrix (Robot)

L0 = Robot.getBody("panda_link0");
L1 = Robot.getBody("panda_link1");
L2 = Robot.getBody("panda_link2");
L3 = Robot.getBody("panda_link3");
L4 = Robot.getBody("panda_link4");
L5 = Robot.getBody("panda_link5");
L6 = Robot.getBody("panda_link6");
L7 = Robot.getBody("panda_link7");
L8 = Robot.getBody("panda_link8");

% m = []
L = [L0, L1, L2, L3, L4, L5, L6, L7]

m = [L0.Mass, L1.Mass, L2.Mass, L3.Mass, L4.Mass, L5.Mass, L6.Mass, L7.Mass];

G = zeros(6,6,length(L));

for i= 1:length(L)

    G(1, 1, i) = L(i).Inertia(1); %m(i) * (3 * r(i)^2 + l(i)^2) / 12;
    G(2, 2, i) = L(i).Inertia(2); %m(i) * (3 * r(i)^2 + l(i)^2) / 12;
    G(3, 3, i) = L(i).Inertia(3); %m(i) * r(i)^2 / 2;
    G(4, 4, i) = L(i).Inertia(4); %m(i);
    G(5, 5, i) = L(i).Inertia(5); %m(i);
    G(6, 6, i) = L(i).Inertia(6); %m(i);
end


end
