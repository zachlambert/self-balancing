function K = get_controller(p1, p2, p3, p4)
    [lambda2, omega2] = get_parameters();

    a0 = -p1*p2*p3;
    a1 = p1*p2 + p1*p3 + p2*p3;
    a2 = -(p1 + p2 + p3);

    k11 = (-a1 + lambda2)/omega2;
    k12 = -(a2*lambda2 + a0)/(omega2*lambda2);
    k13 = -a0/lambda2;
    k24 = -p4;

    K = [
        k11, k12, k13, 0;
        0, 0, 0, k24
    ];
end