% Code for the cost function given a waypoint

function [cost] = cost_function_mult_waypoint(waypoint1, waypoint2, v_max)
    
    L = 10;

    x_1 = waypoint1(1);
    x_2 = waypoint2(1);
    
    y_1 = waypoint1(2);
    y_2 = waypoint2(2);
    
    t_1 = waypoint1(3);
    
    if ((x_1 > 0) && (x_1 < 10) && (y_1 > 0) && (y_1 < 10))
        cost = sin(sqrt((x_1-5)^2+(y_1-5)^2)) / sqrt((x_1-5)^2+(y_1-5)^2);
        %cost = sin(2*x) + sin(2*y) + 3;
    else
        cost = 10;
    end
    
    v_req = sqrt((x_1 - x_2)^2 + (y_1 - y_2)^2) / t_1;
    
    if (v_req <= v_max)
        cost = cost + exp(-(v_req - v_max));
    else
        cost = cost + L + (v_req - v_max) * L^2;
    end
    
end