% Will only work if you've flushed previous data from the
% serial port, and the first new data is from the robot
% flush(s) doesn't work
% Can do: cat /dev/ttyUSB0

s = serialport("/dev/ttyUSB0", 57600);
N = 80*4;
data = read(s, N, "int32");
% Should need to just divide by 10000, but for some reason the values
% matlab gives back are 40000 times larger.
data = data / 10000;
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