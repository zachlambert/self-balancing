% Will only work if you've flushed previous data from the
% serial port, and the first new data is from the robot
% flush(s) doesn't work
% Can do: cat /dev/ttyUSB0

s = serialport("/dev/ttyUSB0", 57600);
flush(s);
M = 8;
N = 400*M;
data = read(s, N, "int32");
% Should need to just divide by 10000, but for some reason the values
% matlab gives back are 40000 times larger.
data = data / 10000;
delete(s);
clear s;
t = data(1:M:N);
theta = data(2:M:N);
theta_dot = data(3:M:N);
phi_dot = data(4:M:N);
psi_1_dot = data(5:M:N);
psi_2_dot = data(6:M:N);
cmd_1_dot = data(7:M:N);
cmd_2_dot = data(8:M:N);
hold on;
plot(t, theta);
plot(t, theta_dot);
plot(t, phi_dot);
plot(t, psi_1_dot);
plot(t, psi_2_dot);
plot(t, cmd_1_dot);
plot(t, cmd_2_dot);
legend("\theta", "\theta vel", "\phi vel", "\psi_1 vel", "\psi_2 vel", "cmd 1", "cmd 2");