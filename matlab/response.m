[A, B, C, D] = get_model();
sys = ss(A, B, C, D);
[K, S, E] = lqr(sys, eye(4), 0.1*eye(2), zeros(4,2));
[U, Sigma] = eig(A - B*K)
K

N = 1000;
T = 5;

x0 = [0.1; 0; 0; 0];
t = linspace(0, T, N);
x = zeros(4, N);
y = zeros(4, N);

for i = 1:N
    x(:, i) = expm((A-B*K)*t(i))*x0;
    y(:, i) = C*x(:, i);
end

[lambda2, omega2, eta1, eta2] = get_parameters();

hold on;
plot(t, x(1,:));
plot(t, x(2,:));
plot(t, x(3,:));
plot(t, x(4,:));
plot(t, y(3,:)/eta1);
plot(t, y(4,:)/eta2);
legend("\theta", "\theta vel", "V", "\Omega", "PWM_1", "PWM_2");