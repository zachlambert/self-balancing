[A, B, C, D] = get_model();
K = get_controller(-1+0.5i, -1-0.5i, -0.5, -1);
display(K(1,1));
display(K(1,2));
display(K(1,2));
display(K(2,4));
K = [
    -500, -200, -10, 0;
    0, 0, 0, 2
];

N = 1000;
T = 5;

x0 = [0.1; 0; 0; 0];
t = linspace(0, T, N);
x = zeros(4, N);
for i = 1:N
    x(:, i) = expm((A-B*K)*t(i))*x0;
end

hold on;
plot(t, x(1,:));
plot(t, x(2,:));
plot(t, x(3,:));
plot(t, x(4,:));
legend("\theta", "\theta vel", "a vel", "b vel");