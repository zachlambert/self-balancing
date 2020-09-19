[A, B, C, D] = load_model();

x0 = [0.2; 0; 0; 0];
u = [0];

T = 1;
N = 1000;
t = linspace(0, T, N);
dt = t(2) - t(1);
x = zeros(4, N);

x(:,1) = transpose(x0);

for i = 2:N
    next = rk4_step(x(:, i-1), u, A, B, C, D, dt);
    x(:, i) = transpose(next);
end

plot(t, x(1,:));
