clc;clear all;
% myev3 = legoev3('wifi','192.168.43.115','74da38bd7f9f');
myev3 = legoev3('usb');
ls = 0
rs = 0
%Initialization of the Ports 
lm = motor(myev3,'A');
rm = motor(myev3,'D');
error = ls -rs
while(true)
    %lm.start();
    %rm.start();
    ls = readInputDeviceREADY_RAW(myev3,4,0,1)-40;
    ms = readInputDeviceREADY_RAW(myev3,3,0,1);
    rs = readInputDeviceREADY_RAW(myev3,2,0,1)+40;
    error = ls-rs
    if(error > 0)
        lm.Speed = 50;
        rm.Speed = 10;
    elseif(error < 0)
        rm.Speed = 50;
        lm.Speed = 10;
    else
        lm.Speed = 50;
        rm.Speed = 50;
    end
    pause(0.5)
end