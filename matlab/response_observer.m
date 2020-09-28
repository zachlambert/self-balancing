[A, B, C] = get_model_observer();
[~, ~, eta1, eta2, ~, ~] = get_parameters();

a1 = -750;
a2 = -150;
a3 = -40;
b4 = 4;
K = [
    a1/eta1, a2/eta1, a3/eta1, b4/eta1, 0;
    a1/eta2, a2/eta2, a3/eta2, -b4/eta2, 0;
];

Cinv = inv(C(2:4, 2:4));

L11 = 5;
L22 = 5;
L51 = 5;
L52 = 0;
Lvel = 0;
Ltop = [
    L11 0 0 0
];
Lmid = [[0;0;0], diag([L22, Lvel, Lvel], 0) * Cinv];
Lbot = [L51 L52 0 0];
L = [Ltop; Lmid; Lbot];
L
eig(A - L*C)

N = 1000;
T = 5;

bias = 0.2;
x0 = [0.1; 0; 0; 0; bias];
x_hat_0 = zeros(5, 1);
x0 = [x0; x0 - x_hat_0];
t = linspace(0, T, N);
x = zeros(10, N);
y = zeros(4, N);

G = [
    A - B*K B*K
    zeros(5, 5) A-L*C
];

for i = 1:N
    x(:, i) = expm(G*t(i))*x0;
    y(:, i) = [C, zeros(4, 5)]*x(:, i);
end

hold on;
plot(t, x(1,:));
plot(t, x(2,:));
plot(t, x(6,:));
plot(t, x(7,:));
plot(t, x(10,:));
legend("\theta", "\theta vel", "\theta error", "\theta vel error", "bias error");