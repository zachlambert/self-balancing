function [A, B, C, D] = get_model()
    [lambda2, omega2, eta1, eta2, R, D] = get_parameters();
    A = [
        0 1 0 0
        lambda2 0 0 0
        0 R 0 0
        0 0 0 0
    ];
    B = [
        0 0
        -0.5*omega2*eta1 -0.5*omega2*eta2
        0.5*eta1*R 0.5*eta2*R
        0.5*eta1*R/D -0.5*eta2*R/D
    ];
    C = [
        1 0 0 0
        0 1 0 0
        0 0 1/(R*eta1) D/(R*eta1)
        0 0 1/(R*eta2) -D/(R*eta2)
    ];
    D = zeros(4, 4);
end
