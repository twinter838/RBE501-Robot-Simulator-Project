function traj = make_trajectory(type, params)

    q0 = params.q0
    qf = params.q1
    v0 = params.v0
    vf = params.v1
    t0 = params.t0
    tf = params.t1
    dt = params.dt

    if strcmp(type, 'cubic')
        A = [0      0       t0^3    t0^2    t0      1;
             0      0       3*t0^2  2*t0    1       0;
             0      0       0       0       0       0;
             0      0       tf^3    tf^2    tf      1;
             0      0       3*tf^2  2*tf    1       0;
             0      0       0       0       0       0;];
        
        a0 = 0;
        af = 0;
        
    elseif strcmp(type, 'quintic')
        A = [t0^5   t0^4    t0^3    t0^2    t0      1;
             5*t0^4  4*t0^3  3*t0^2  2*t0   1       0;
             20*t0^3 12*t0^2 6*t0    2      0       0;
             tf^5   tf^4    tf^3    tf^2    tf      1;
             5*tf^4  4*tf^3  3*tf^2  2*tf   1       0;
             20*tf^3 12*tf^2 6*tf    2      0       0;];

        a0 = params.a0
        af = params.a1

    B = [q0 v0 a0 qf vf af]';
    X = pinv(A) * B;


    t = transpose(t0:dt:tf);

    q = [t.^5   t.^4    t.^3    t.^2    t       t.^0] * X;
    v = [5*t.^4  4*t.^3  3*t.^2  2*t    t.^0    t*0] * X;
    a = [20*t.^3 12*t.^2 6*t    2*t.^0  0*t     0*t] * X;

    traj.t = t;
    traj.q = q;
    traj.v = v;
    traj.a = a;

end