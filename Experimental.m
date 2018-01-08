%pause(3);
clc;clear all;
% myev3=legoev3('wifi','192.168.43.115','74da38bd7f9f');
myev3 = legoev3('usb');

%Initialization of ll  the Ports 
ls = 0;
ms = 0;
rs = 0;
%us = readInputDeviceREADY_RAW(myev3,4,0,1);
us = 50000;
lm = motor(myev3,'D');
rm = motor(myev3,'A');

%Setting the Bias and cutoff Values for the Sensors
%%% TODO
%%% Calibrate and set the threshold for each sensors
us_threshold = 25;
ls_bias = 0;
ms_bias = 0;
rs_bias = 0;
set_point = 0;
st_speed = 100;
cont_speed = 50;

%%% PID Parameters
kp = 0.9;
ki = 0;
kd = 1;
prev_error = [0,0];
i_error = [0,0];
sampling_time =0.001;
out_count = 0;
st_line_count = 0;
setp_l = 3308;
setp_r = 3292;
iter = 1;
a = [];
lm.start();
rm.start();
while(true)
    ls = readInputDeviceREADY_RAW(myev3,4,0,1);
    ms = readInputDeviceREADY_RAW(myev3,3,0,1);
    rs = readInputDeviceREADY_RAW(myev3,2,0,1);
    us = range_converter(readInputDeviceREADY_RAW(myev3,1,0,1),66,166,2,20);
    disp("Distance: "+us);
    if(us < us_threshold)
        lm.stop();
        rm.stop();
        break
    end
    if(ms>3250 && ms < 3360 && ls>3250 && ls < 3360 && rs>3250 && rs < 3360)
        disp("Detected Red")
        lm.stop()
        rm.stop()
        break;
    end
    if(out_count >= 50)
        lm.stop()
        rm.stop()
        break;
    end
    if(ls>3300 && rs >3300)
        out_count = out_count + 1;
    else
        out_count = 0;
    end
    disp("Out_Count: "+out_count);
    cum_error = [ls + ls_bias - setp_l,rs + rs_bias - setp_r];
    
    error = [min(100,max(-100,range_converter(cum_error(1),-277,277,-100,100))),min(100,max(-100,range_converter(cum_error(2),-277,277,-100,100)))];
    disp("Error: "+cum_error)
    d_error = [error(1)-prev_error(1),error(2)-prev_error(2)];
    i_error = [min(20,max(-20,error(1)+i_error(1))),min(20,max(-20,error(2)+i_error(2)))];
    
    mv_l = kp * error(1) + ki*i_error(1) + kd*d_error(1);
    mv_r = kp * error(2) + ki*i_error(2) + kd*d_error(2);
    
    disp("motor Correction: "+mv_l+","+mv_r)
    
    lm.Speed = st_speed - mv_l;
    rm.Speed = st_speed + mv_r;
    disp("Motor Speed: "+lm.Speed+","+rm.Speed);
    prev_error = [error(1),error(2)];
end
