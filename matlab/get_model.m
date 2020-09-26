function [A, B, C, D] = get_model()
    [lambda2, omega2] = get_parameters();
    A = [
        0 1 0 0
        lambda2 0 0 0
        0 0 0 0
        0 0 0 0
    ];
    B = [
        0 0
        -omega2 0
        1 0
        0 1
    ];
    C = eye(4);
    D = zeros(4, 4);
end
