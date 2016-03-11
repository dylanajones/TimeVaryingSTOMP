% Function to calculate the time between two way points
% This could be done with norm(w1 - w2) / v_max


function [t] = my_distance(w1, w2, vmax)
    x_dist = w1(1) - w2(1);
    y_dist = w1(2) - w2(2);
    
    tot_dist = sqrt(x_dist^2 + y_dist^2);
    
    t = tot_dist / vmax;
end