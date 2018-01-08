function y=range_converter(x,x_min,x_max,y_min,y_max)
%     y = ((x-x_min)/(x_max-x_min))*(y_max-y_min) - y_min;  
    y = (((x - x_min) * (y_max-y_min))/ (x_max-x_min)) + y_min;
end
