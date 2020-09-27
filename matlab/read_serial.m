% Will only work if you've flushed previous data from the
% serial port, and the first new data is from the robot
% flush(s) doesn't work
% Can do: cat /dev/ttyUSB0

s = serialport("/dev/ttyUSB0", 57600);
flush(s);
M = 9;
N = 100*M;
data = read(s, N, "int32");
% Should need to just divide by 10000, but for some reason the values
% matlab gives back are 40000 times larger.
data = data / 10000;
delete(s);
clear s;
t = data(1:M:N);
pwm1 = data(2:M:N);
pwm2 = data(3:M:N);
u1 = data(4:M:N);
u2 = data(5:M:N);
y1 = data(6:M:N);
y2 = data(7:M:N);
y3 = data(8:M:N);
y4 = data(9:M:N);

hold on;
plot(t, pwm1);
plot(t, pwm2);
plot(t, u1);
plot(t, u2);
plot(t, y1);
plot(t, y2);
plot(t, y3);
plot(t, y4);
legend("PWM_1", "PWM_2", "u_1", "u_2", "\theta vel", "\theta vel", "\psi_1 vel", "\psi_2 vel");