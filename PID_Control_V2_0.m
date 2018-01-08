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
us_threshold = 10;
ls_bias = 10;
ms_bias = 0;
rs_bias = -50;
set_point = 0;
st_speed = 80;

%%% PID Parameters
kp = 0.25;
ki = 0.2;
kd = 0.1;
prev_error = 0;
i_error = 0;
sampling_time =0.005;
out_count = 0;
st_line_count = 0;
iter = 1;
a = [];
stable_count = 0;
lm.start();
rm.start();
while(true)  
    disp("Current Out_Count = "+out_count)
    ls = readInputDeviceREADY_RAW(myev3,4,0,1);
    ms = readInputDeviceREADY_RAW(myev3,3,0,1);
    rs = readInputDeviceREADY_RAW(myev3,2,0,1);
    disp("Stability Index: "+stable_count);
    cum_sensor = (ls + ls_bias) - (rs+rs_bias);
    a(iter) = ls+ls_bias;
    disp("Actual Error: "+cum_sensor);
    error = min(100,max(-100,range_converter(cum_sensor,-320,450,-100,100)));
    if(abs(error) <= 50)
        stable_count = stable_count + 1;
    else
        stable_count = 0;
    end
    if(stable_count < 3)
        kp = 0.25;
        ki = 0.02;
        kd = 0.1;
        st_speed = 80;
        disp("Going Straight");
    else
        kp = 0.45;
        ki = 0.2;
        kd = 0.01;
        st_speed = 80;
        disp("Taking a turn");
    end
    cont_speed = st_speed;
    us = range_converter(readInputDeviceREADY_RAW(myev3,1,0,1),66,166,2,20);
    disp("Distance: "+us);
    if(mod(iter,10) == 0)
        if(abs(error) > 80)
            stable_count = 5;
        else
            stable_count = 0;
        end
    end
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
    disp("Corrected Error: "+error);
    d_error = (error - prev_error);
    disp("Rate of Change of Error: "+d_error);
    i_error = min(50,max(-50,i_error + error));
%     i_error = i_error + error;
    disp("Proportional_part: "+kp * error);
    disp("Integral_part: "+ki*i_error);
    disp("Derivative_part: "+kd*d_error);
    mv = kp*error +kd*d_error +ki*i_error;
    disp("Corrective_action: "+mv)
    if(out_count >= 5)
        lm.stop()
        rm.stop()
        break;
    end
    if(ls>3300 && rs >3300)
        out_count = out_count + 1;
    else
        out_count = 0;
    end
    if(mv<0)
        motor_correction = min(cont_speed,max(0,abs(mv)));
        if(motor_correction < 5)
            st_line_count = st_line_count + 1;
        else
            st_line_count = 0;
        end
        if(st_line_count > 5)
            lm.Speed = st_speed - motor_correction + 30;
            rm.Speed = st_speed + motor_correction +30;
        else
            lm.Speed = st_speed - motor_correction;
            rm.Speed = st_speed;
        end
        disp("Motor Turning left: "+lm.Speed +","+rm.Speed)
    else
        motor_correction = min(cont_speed,max(0,abs(mv)));
        if(motor_correction < 5)
            st_line_count = st_line_count + 1;
        else
            st_line_count = 0;
        end
        if(st_line_count > 5)
            rm.Speed = st_speed - motor_correction + 30;
            lm.Speed = st_speed + motor_correction + 30;
        else
            rm.Speed = st_speed - motor_correction;
            lm.Speed = st_speed;
        end
        disp("Motor Turning right: "+lm.Speed +","+rm.Speed)
    end
    prev_error = error;
    pause(sampling_time);
    iter = iter + 1;
    disp("----------------------------------------------------------------")
end
lm.stop();
rm.stop();



