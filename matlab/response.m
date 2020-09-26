[A, B, C, D] = get_model();
K = [
    -500, -200, -10, 0;
    0, 0, 0, 2
];
display(eig(A - B*K));

L = zeros(5, 4);
display(eig(A - L*C));

N = 1000;
T = 5;

theta_0 = 0.1;
bias = 0.2;
x_0 = [theta_0; 0; 0; bias];
x_hat_0 = zeros(5);
t = linspace(0, T, N);

for i = 1:N
    x(:, i) = expm((A-B*K)*t(i))*x0;
end

hold on;
plot(t, x(1,:));
plot(t, x(2,:));
plot(t, x(3,:));
plot(t, x(4,:));
legend("\theta", "\theta vel", "a vel", "b vel");