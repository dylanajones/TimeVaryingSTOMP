% Function to do Cost Function with Currents

function [cost] = cost_with_currents_expectation(w1, w2, w3, u, v, v_max, size)
    
    % Constants for calculations
    cost = 0;
    L = 10;
    factor = 5;
    c_d = 3;
    
    % Making the points
    x_1 = w1(1);
    y_1 = w1(2);
    t_1 = w1(3);
    
    x_2 = w2(1);
    y_2 = w2(2);
    t_2 = w2(3);
    
    x_3 = w3(1);
    y_3 = w3(2);
    
    % Calculating distances between points
    dist_12 = sqrt((x_1 - x_2)^2 + (y_1 - y_2)^2)
    dist_23 = sqrt((x_2 - x_3)^2 + (y_2 - y_3)^2)
    
    t_13 = t_1 + dist_23 / dist_12 * t_1
    
    % Translating the waypoint location to our discritized current field

    x_per_1 = x_1 / size(1);
    y_per_1 = y_1 / size(2);
    
    x_per_2 = x_2 / size(1);
    y_per_2 = y_2 / size(2);
    
    x_per_3 = x_3 / size(1);
    y_per_3 = y_3 / size(2);
    
    x_index_1 = int8(x_per_1 * length(u))
    
    if x_index_1 < 1
        x_index_1 = 1;
    end
    
    y_index_1 = int8(y_per_1 * length(u))
    
    if y_index_1 < 1
        y_index_1 = 1;
    end
    
    x_index_2 = int8(x_per_2 * length(u))
    
    if x_index_2 < 1
        x_index_2 = 1;
    end
    
    y_index_2 = int8(y_per_2 * length(u))
    
    if y_index_2 < 1
        y_index_2 = 1;
    end
    
    x_index_3 = int8(x_per_3 * length(u))
    
    if x_index_3 < 1
        x_index_3 = 1;
    end
    
    y_index_3 = int8(y_per_3 * length(u))
    
    if y_index_3 < 1
        y_index_3 = 1;
    end
    
    % Calculating the average current between the two way points
    u_avg_12 = (u(x_index_1, y_index_1) + u(x_index_2, y_index_2)) / 2
    v_avg_12 = (v(x_index_1, y_index_1) + v(x_index_2, y_index_2)) / 2
    
    u_avg_23 = (u(x_index_2, y_index_2) + u(x_index_3, y_index_3)) / 2
    v_avg_23 = (v(x_index_2, y_index_2) + v(x_index_3, y_index_3)) / 2
    
    u_avg_13 = (u(x_index_1, y_index_1) + u(x_index_3, y_index_3)) / 2
    v_avg_13 = (v(x_index_1, y_index_1) + v(x_index_3, y_index_3)) / 2
    
    % Calculating the required velocity to travel between the points
    u_req_12 = (x_1 - x_2) / t_1 - u_avg_12
    v_req_12 = (y_1 - y_2) / t_1 - v_avg_12
    
    u_req_23 = (x_2 - x_3) / t_2 - u_avg_23
    v_req_23 = (y_2 - y_3) / t_2 - v_avg_23
    
    u_req_13 = (x_1 - x_3) / t_13 - u_avg_13
    v_req_13 = (y_1 - y_3) / t_13 - v_avg_13
   
    abs_vel_12 = sqrt(((x_1 - x_2) / t_1)^2 + ((y_1 - y_2) / t_1)^2)
    abs_vel_23 = sqrt(((x_2 - x_3) / t_2)^2 + ((y_2 - y_3) / t_2)^2)
    abs_vel_13 = sqrt(((x_1 - x_3) / t_13)^2 + ((y_1 - y_3) / t_13)^2)
    
    vel_req_12 = sqrt(u_req_12^2 + v_req_12^2)
    vel_req_23 = sqrt(u_req_23^2 + v_req_23^2)
    vel_req_13 = sqrt(u_req_13^2 + v_req_13^2)
    
    if (vel_req_23 <= v_max && abs_vel_23 > v_max)
        cost = 1;
    elseif (vel_req_23 <= v_max)
        cost = cost + factor * exp(-(vel_req_23 - v_max));
    else
        cost = cost + L + (vel_req_23 - v_max) * L^2;
    end
    
    % Adding in an energy cost
    % cost = cost + c_d * vel_req ^ 3 * t_1;
    
    % Energy cost with waypoint
    cost_with = c_d * vel_req_12 ^ 3 * t_1 + c_d * vel_req_23 ^ 3 * t_2;
    
    % Energy cost without waypoint
    cost_without = c_d * vel_req_13 ^ 3 * t_13;
    
    % Adding in energy cost
    cost = cost + exp(cost_with - cost_without)
    
end






























