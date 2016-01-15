% Function to do Cost Function with Currents

function [cost] = cost_with_currents(w1, w2, u, v, v_max, size)
    % Constants for calculations
    cost = 0;
    L = 10;
    factor = 5;
    c_d = 3;
    
    % Making the points
    x_1 = w1(1);
    y_1 = w1(2);
    t = w1(3);
    
    x_2 = w2(1);
    y_2 = w2(2);
    
    % Translating the waypoint location to our discritized current field
    x_per_1 = x_1 / size(1);
    y_per_1 = y_1 / size(2);
    
    x_per_2 = x_2 / size(1);
    y_per_2 = y_2 / size(2);
    
    x_index_1 = int8(x_per_1 * length(u));
    
    if x_index_1 < 1
        x_index_1 = 1;
    end
    
    y_index_1 = int8(y_per_1 * length(u));
    
    if y_index_1 < 1
        y_index_1 = 1;
    end
    
    x_index_2 = int8(x_per_2 * length(u));
    
    if x_index_2 < 1
        x_index_2 = 1;
    end
    
    y_index_2 = int8(y_per_2 * length(u));
    
    if y_index_2 < 1
        y_index_2 = 1;
    end
    
    % Calculating the average current between the two way points
    u_avg = (u(x_index_1, y_index_1) + u(x_index_2, y_index_2)) / 2;
    v_avg = (v(x_index_1, y_index_1) + v(x_index_2, y_index_2)) / 2;
    
    % Calculating the required velocity to travel between the points
    u_req = (x_1 - x_2) / t - u_avg;
    v_req = (y_1 - y_2) / t - v_avg;
    
    abs_vel = sqrt(((x_1 - x_2) / t)^2 + ((y_1 - y_2) / t)^2);
    
    vel_req = sqrt(u_req^2 + v_req^2);
    
    if (vel_req <= v_max && abs_vel > v_max)
        cost = 1;
    elseif (vel_req <= v_max)
        cost = cost + factor * exp(-(vel_req - v_max));
    else
        cost = cost + L + (vel_req - v_max) * L^2;
    end
    
    % Adding in an energy cost
    cost = cost + c_d * vel_req ^ 3 * t;
    
end









