clc;clear all;

% myev3.delete();
% myev3=legoev3('wifi','192.168.43.115','74da38bd7f9f');
myev3 = legoev3('usb');

condition=true;
while(condition)
       us = readInputDeviceREADY_RAW(myev3,1,0,1);
       ls = readInputDeviceREADY_RAW(myev3,4,0,1);
       ms = readInputDeviceREADY_RAW(myev3,3,0,1);
       rs = readInputDeviceREADY_RAW(myev3,2,0,1);
       disp("ultrasonic Sensor: ")
       temp = range_converter(us,66,166,2,20)
       disp("left Sensor: ")
       temp = double(ls)
       disp("Error: ")
       temp =  ls + 15 - rs -45
       disp("Middle Sensor: ")
       temp = double(ms)
       disp("Right Sensor: ")
       temp = double(rs)
       pause(0.5);
end