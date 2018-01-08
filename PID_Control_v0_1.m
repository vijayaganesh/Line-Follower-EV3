%pause(3);
clc;clear all;
file = "ECE556-group13_task1_trial3.txt";
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
us_threshold = 16;
ls_bias = 0;
ms_bias = 0;
rs_bias = 0;
set_point = 0;
st_speed = 85;
cont_speed = 100;

%%% PID Parameters
kp = 0.29;
ki = 0.1;
kd = 0.05;
prev_error = 0;
i_error = 0;
sampling_time =0.001;
out_count = 0;
st_line_count = 0;
iter = 1;
a = [];
lm.start();
rm.start();
prev_ls = 0;
prev_rs = 0;
prev_ls_1 = 0;
prev_rs_1 = 0;
rs = readInputDeviceREADY_RAW(myev3,2,0,1);
last_anchor = readInputDeviceREADY_RAW(myev3,2,0,1);
line_iter = 0;
tic;
file_data = [];
file_data(line_iter+1,1)=0;
file_data(line_iter+1,2)=rs;
line_iter = line_iter + 1;
next_time = 0.05;
while(true) 
    disp("Prev_sensor_Reading: "+prev_ls+","+prev_rs);
    disp("Prev_sensor_Reading(Lag 1): "+prev_ls_1+","+prev_rs_1);
    disp("Current Out_Count = "+out_count)
    ls = readInputDeviceREADY_RAW(myev3,4,0,1);
    ms = readInputDeviceREADY_RAW(myev3,3,0,1);
    rs = readInputDeviceREADY_RAW(myev3,2,0,1);
    t = toc
%     if(t <= next_time+0.025 && t >= next_time-0.025)
%         file_data(line_iter+1,1)=next_time;
%         file_data(line_iter+1,2)=rs;
%         line_iter = line_iter + 1;
%         
%     end
        file_data(line_iter+1,1)=t;
        file_data(line_iter+1,2)=rs;
        line_iter = line_iter + 1;


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
    cum_sensor = (ls + ls_bias) - (rs+rs_bias);
    disp("Actual Error: "+cum_sensor);
    error = min(100,max(-100,range_converter(cum_sensor,-380,380,-100,100)));
    disp("Corrected Error: "+error);
    d_error = (error - prev_error);
    disp("Rate of Change of Error: "+d_error);
    i_error = min(50,max(-50,i_error + error));
%     i_error = i_error + error;
    disp("Proportional_part: "+kp * error);
    disp("Integral_part: "+ki*i_error);
    disp("Derivative_part: "+kd*d_error);
%     mv = kp*error + kd*(prev_error-error)/sampling_time + ki*(prev_mv+(prev_error+error)/2)*sampling_time;
    mv = kp*error +kd*d_error +ki*i_error;
    disp("Corrective_action: "+mv)
%     adj_mv = min(500,max(-500,mv));
%     disp(adj_mv)
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
    iter = iter + 1;
    if(out_count >= 40)
        lm.stop()
        rm.stop()
        break;
    end
    if(ls>3300 && rs >3300)
        out_count = out_count + 1;
        if(prev_ls_1>prev_rs_1)
            lm.Speed = 80;
            rm.Speed = -80;
        else
            lm.Speed = -80;
            rm.Speed = 80;
        end
        continue;
    else
        out_count = 0;
    end
    prev_rs_1 = prev_rs;
    prev_ls_1 = prev_ls;
    prev_rs = rs;
    prev_ls = ls;
    disp("----------------------------------------------------------------")
    
end
lm.stop();
rm.stop();
file_id = fopen(file,'w');
for i=1:length(file_data)
    fprintf(file_id,'@%d@%f\n',file_data(i,2),file_data(i,1));
end
fclose(file_id);




