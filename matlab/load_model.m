function [A, B, C, D] = load_model()
    g = 9.81;
    R = 0.25;
    L = 0.06;
    mb = 0.1;
    mw = 0.02;
    Ib = 0.1*0.1^2;
    Iw = mw*R^2/2;
    k = 5;

    M = [
        mb*L^2 + Ib + Iw, mb*R*L + Iw;
        mb*R*L + Iw, mb*R^2 + mw*R^2 + Iw
    ];
    K = [
        -mb*g*L, 0;
        0, 0
    ];

    A = [
        zeros(2, 2), eye(2);
        -inv(M)*K, zeros(2, 2)
    ];
    B = [0; 0; (inv(M)*[0; 1])];
    C = [
        1, 0, 0, 0;
        0, 0, 1, 0;
        0, 0, 0, 1
    ];
    D = zeros(3, 1);
end
