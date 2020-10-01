[A, B, C] = get_model();
[lambda2, omega2, eta1, eta2, R, D] = get_parameters();

a3 = -300;
a2 = a3*R/omega2 - 400;
a1 = lambda2*a2 / (a3*R - omega2*a2) - a3*R - 12;
b4 = 200;

K = [
    a1/eta1, a2/eta1, a3/eta1, b4/eta1;
    a1/eta2, a2/eta2, a3/eta2, -b4/eta2;
];
[U, Sigma] = eig(A - B*K)

% Always have an eigenvector [0, 0, 0, 1], ith eigenvalue
% equal to b4 * R/D, since rotational acceleration is independent
% from the tilt/forward velocity motion.

