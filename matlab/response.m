[A, B, C, D] = get_model();
K = [
    -25, -2, -1, 2;
    -25, -2, -1, 2;
];
display(eig(A - B*K));

N = 1000;
T = 5;

x_0 = [0.1; 0; 0; 0];
t = linspace(0, T, N);
x = zeros(4, N);
y = zeros(4, N);

for i = 1:N
    x(:, i) = expm((A-B*K)*t(i))*x0;
    y(:, i) = C*x(:, i);
end

hold on;
plot(t, x(1,:));
plot(t, x(2,:));
plot(t, x(3,:));
plot(t, x(4,:));
plot(t, y(3,:));
plot(t, y(4,:));
legend("\theta", "\theta vel", "V", "\Omega", "\phi_1 vel", "\phi_2 vel");