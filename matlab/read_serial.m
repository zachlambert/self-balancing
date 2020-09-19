s = serialport("/dev/ttyUSB0", 9600);
data = read(s, 50, "int32");
data = data / 1000000;
delete(s);
clear s;
plot(data);