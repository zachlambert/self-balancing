[A, B, C] = get_model();
[lambda2, omega2, eta1, eta2, R, D] = get_parameters();

% Set poles manually
p1 = -10+0.5i;
p2 = -10-0.5i;
p3 = -2;
p4 = -2;
alpha0 = -p1*p2*p3;
alpha1 = p1*p2 + p1*p3 + p2*p3;
alpha2 = -(p1 + p2 + p3);

% Set poles by choosing values just in the valid
% range for routh hurwitz
a3 = 0 - 300;
a2 = a3*R/omega2 - 400;
a1 = lambda2*a2 / (a3*R - omega2*a2) - a3*R - 400;
b4 = 20;

a1
a2
a3
b4

K = [
    a1/eta1, a2/eta1, a3/eta1, b4/eta1;
    a1/eta2, a2/eta2, a3/eta2, -b4/eta2;
];
eig(A - B*K)
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