function S = screw(v,p)
%Screw Assembles a screw axis from its two components
S=-skew(v)*p