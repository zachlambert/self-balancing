s = serialport("/dev/ttyUSB0", 9600);
N = 20*4;
data = read(s, N, "int32");
% Should need to just divide by 10000, but for some reason the values
% matlab gives back are 40000 times larger.
data = data / 4e8;
delete(s);
clear s;
t = data(1:4:N);
theta = data(2:4:N);
theta_dot = data(3:4:N);
phi_dot = data(4:4:N);
hold on;
plot(t, theta);
plot(t, theta_dot);
plot(t, phi_dot);