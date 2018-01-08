clear all;
% myev3 = legoev3('wifi','192.168.43.115','74da38bd7f9f');
myev3 = legoev3('usb');
mymotor1=motor(myev3,'D');
mymotor1.Speed=75;
mymotor2 = motor(myev3,'A');
mymotor2.Speed=75;
condition=true;
count = 0;
mymotor1.start();
mymotor2.start();
while(count<10)
    speed = 1+74*rand(1,1);
    mymotor1.Speed = speed;
    mymotor2.Speed = speed;
    pause(1)

    count = count + 1;
end
mymotor1.stop();
mymotor2.stop();
