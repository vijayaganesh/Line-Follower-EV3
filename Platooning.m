%pause(3);
clc;clear all;
file = "ECE556-group13_task2_trial3.txt";
% myev3=legoev3('wifi','192.168.43.115','74da38bd7f9f');
myev3 = legoev3('usb');

%Initialization of ll  the Ports
ls = 0;
ms = 0;
rs = 0;
ls_bias = 0;
ms_bias = 0;
rs_bias = -40;
kp_line = 0.2;
kp_dist = 0.65;
kd_dist = 5;
out_count = 0;
%us = readInputDeviceREADY_RAW(myev3,4,0,1);
set_point = 20;
lm = motor(myev3,'D');
rm = motor(myev3,'A');
st_speed = 0;
max_speed = 85;
cont_speed = 0;
sampling_time = 0.0001;
lm.start();
rm.start();
us = readInputDeviceREADY_RAW(myev3,1,0,1);
prev_error = 0;
line_iter = 0;
tic;
file_data = [];
file_data(line_iter+1,1)=0;
file_data(line_iter+1,2)=us;
line_iter = line_iter + 1;
next_time = 0.05;
while(true)
    ls = readInputDeviceREADY_RAW(myev3,4,0,1);
    ms = readInputDeviceREADY_RAW(myev3,3,0,1);
    rs = readInputDeviceREADY_RAW(myev3,2,0,1);
    us = readInputDeviceREADY_RAW(myev3,1,0,1);
    t = toc
%     if(t <= next_time+0.015)
%         file_data(line_iter+1,1)=next_time;
%         file_data(line_iter+1,2)=range_converter(us,66,166,2,20);
%         line_iter = line_iter + 1;
%         next_time = next_time + 0.05;
%     end
    file_data(line_iter+1,1)=t;
    file_data(line_iter+1,2)=range_converter(us,66,166,2,20);
    line_iter = line_iter + 1;
    if(ms>3300 && ms < 3460 && ls>3300 && ls < 3460 && rs>3300 && rs < 3500)
        disp("Detected Red")
        lm.stop()
        rm.stop()
        break;
    end
    cum_sensor = (ls + ls_bias) - (rs+rs_bias);
    line_error =  min(100,max(-100,range_converter(cum_sensor,-320,450,-100,100)));
    curr_dist = range_converter(us,66,166,2,20);
    disp("Current Distance: "+curr_dist);
    dist_error = curr_dist - set_point;
    disp("Distance Correction: "+dist_error)
    d_error = dist_error - prev_error;
    mv = kp_line * line_error;
    disp("Proportional Part says: "+ kp_dist*dist_error);
    dv = kp_dist * dist_error + kd_dist * d_error;
    disp("Derivative Part says: "+ kd_dist*d_error);
    disp("Out Count: "+out_count);
    if(out_count >= 10)
        lm.stop()
        rm.stop()
        break;
    end
    if(ls>3400 && rs >3400)
        out_count = out_count + 1;
    else
        out_count = 0;
    end
    st_speed =min(max_speed,max(0,st_speed + dv -8));
    if(mv<0)
        cont_speed = st_speed * 0.95;
        motor_correction = min(cont_speed,max(0,abs(mv)));
        lm.Speed = min(max_speed,max(0,st_speed - motor_correction));
        rm.Speed = st_speed;
        disp("Motor Turning left: "+lm.Speed +","+rm.Speed)
    else
        cont_speed = st_speed * 0.95;
        motor_correction = min(cont_speed,max(0,abs(mv)));
        rm.Speed = min(max_speed,max(0,st_speed - motor_correction));
        lm.Speed = st_speed;
        disp("Motor Turning right: "+lm.Speed +","+rm.Speed)
    end
    disp("------------------------------------");
    pause(sampling_time);
    prev_error = dist_error;
end
file_id = fopen(file,'w');
for i=1:length(file_data)
    fprintf(file_id,'@%d@%f\n',file_data(i,2),file_data(i,1));
end
