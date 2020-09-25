lambda2 = 0.0389^2;
omega2 = 4;
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

p1 = -2;
p2 = -2;
p3 = -2;

a0 = -p1*p2*p3;
a1 = p1*p2 + p1*p3 + p2*p3;
a2 = -(p1 + p2 + p3);
k24 = 1;

K = [
    (a1 + lambda2)/omega2, (a2*lambda2 + a0)/(omega2*lambda2), a0/lambda2, 0;
    0, 0, 0, -k24
];
K

E = A + B*K;

[U, Sigma] = eig(E);
display(Sigma);