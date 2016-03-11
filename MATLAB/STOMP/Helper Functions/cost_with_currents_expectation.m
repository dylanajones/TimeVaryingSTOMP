% Function to do Cost Function with Currents

function [cost] = cost_with_currents_expectation(w1, w2, w3, u, v, v_max, size)
    
    % Constants for calculations
    cost = 0;
    L = 10;
    factor = 5;
    c_d = 3;
    
    l = length(u);
    
    % Constant for object checking
    O = 100;
    
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
    dist_12 = sqrt((x_1 - x_2)^2 + (y_1 - y_2)^2);
    dist_23 = sqrt((x_2 - x_3)^2 + (y_2 - y_3)^2);
    
    t_13 = t_1 + dist_23 / dist_12 * t_1;
   
    % Translating the waypoint location to our discritized current field
    % Need to add bound checking here
    x_index_1 = int8(l / size(2) * x_1) + 1;
    y_index_1 = int8(l / size(1) * y_1) + 1;
    
    x_index_2 = int8(l / size(2) * x_2) + 1;
    y_index_2 = int8(l / size(1) * y_2) + 1;
    
    x_index_3 = int8(l / size(2) * x_3) + 1;
    y_index_3 = int8(l / size(1) * y_3) + 1;
    
    if x_index_1 > l
        x_index_1 = l;
    end
    
    if x_index_2 > l
        x_index_2 = l;
    end
    
    if x_index_3 > l
        x_index_3 = l;
    end
    
    if y_index_1 > l
        y_index_1 = l;
    end
    
    if y_index_2 > l
        y_index_2 = l;
    end
    
    if y_index_3 > l
        y_index_3 = l;
    end
    
    if (isnan(u(y_index_1, x_index_1)) || isnan(u(y_index_2, x_index_2)) || isnan(u(y_index_3, x_index_3)) || isnan(v(y_index_1, x_index_1)) || isnan(v(y_index_2, x_index_2)) || isnan(v(y_index_3, x_index_3)))
        % Should make this a function of how deep it is into the object
        cost = O^2;
    else
    
        % Calculating the average current between the two way points
        u_avg_12 = (u(y_index_1, x_index_1) + u(y_index_2, x_index_2)) / 2;
        v_avg_12 = (v(y_index_1, x_index_1) + v(y_index_2, x_index_2)) / 2;

        u_avg_23 = (u(y_index_2, x_index_2) + u(y_index_3, x_index_3)) / 2;
        v_avg_23 = (v(y_index_2, x_index_2) + v(y_index_3, x_index_3)) / 2;

        u_avg_13 = (u(y_index_1, x_index_1) + u(y_index_3, x_index_3)) / 2;
        v_avg_13 = (v(y_index_1, x_index_1) + v(y_index_3, x_index_3)) / 2;

        % Calculating the required velocity to travel between the points
        u_req_12 = (x_2 - x_1) / t_1 - u_avg_12;
        v_req_12 = (y_2 - y_1) / t_1 - v_avg_12;

        u_req_23 = (x_3 - x_2) / t_2 - u_avg_23;
        v_req_23 = (y_3 - y_2) / t_2 - v_avg_23;

        u_req_13 = (x_3 - x_1) / t_13 - u_avg_13;
        v_req_13 = (y_3 - y_1) / t_13 - v_avg_13;

        abs_vel_12 = sqrt(((x_2 - x_1) / t_1)^2 + ((y_2 - y_1) / t_1)^2);
        abs_vel_23 = sqrt(((x_3 - x_2) / t_2)^2 + ((y_3 - y_2) / t_2)^2);
        abs_vel_13 = sqrt(((x_3 - x_1) / t_13)^2 + ((y_3 - y_1) / t_13)^2);

        vel_req_12 = sqrt(u_req_12^2 + v_req_12^2);
        vel_req_23 = sqrt(u_req_23^2 + v_req_23^2);
        vel_req_13 = sqrt(u_req_13^2 + v_req_13^2);

        if (vel_req_23 <= v_max && abs_vel_23 >= v_max)
            %display('option 1')
            cost = 0;
        elseif (vel_req_23 <= v_max)
            %display('option 2')
            cost = cost + factor * exp(-(vel_req_23 - v_max));
        else
            %display('option 3')
            cost = cost + L + (vel_req_23 - v_max) * L^2;
        end

        % Adding in an energy cost
        % cost = cost + c_d * vel_req ^ 3 * t_1;

        % Energy cost with waypoint
        cost_with = c_d * vel_req_12 ^ 3 * t_1 + c_d * vel_req_23 ^ 3 * t_2;

        % Energy cost without waypoint
        cost_without = c_d * vel_req_13 ^ 3 * t_13;
    
        % Scaling the difference
        diff = cost_with - cost_without;
        
%         if diff ~= 0
%             diff = diff / (10 ^ floor(log10(abs(diff))));
%         end
        
        % Adding in energy cost
        cost = cost + exp(diff);
    end
    
end






























