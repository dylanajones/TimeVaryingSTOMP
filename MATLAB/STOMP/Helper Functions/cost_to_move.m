% Function to calculate move cost between two waypoints

function [cost] = cost_to_move(w1,w2,u,v,size)
    
    cost = 0;
    c_d = 3;
    
    l = length(u);
       
    % Making the points
    x_1 = w1(1);
    y_1 = w1(2);
    t_1 = w1(3);
    
    x_2 = w2(1);
    y_2 = w2(2);
    t_2 = w2(3);
    
    x_index_1 = int8(l / size(2) * x_1) + 1;
    y_index_1 = int8(l / size(1) * y_1) + 1;
    
    x_index_2 = int8(l / size(2) * x_2) + 1;
    y_index_2 = int8(l / size(1) * y_2) + 1;
    
    if x_index_1 > l
        x_index_1 = l;
    end
    
    if x_index_2 > l
        x_index_2 = l;
    end
    
    if y_index_1 > l
        y_index_1 = l;
    end
    
    if y_index_2 > l
        y_index_2 = l;
    end
    
    if x_index_1 < 1
        x_index_1 = 1;
    end
    
    if x_index_2 < 1
        x_index_2 = 1;
    end
    
    if y_index_1 < 1
        y_index_1 = 1;
    end
    
    if y_index_2 < 1
        y_index_2 = 1;
    end
    
    if isnan(u(y_index_1, x_index_1)) || isnan(v(y_index_1, x_index_1)) || isnan(u(y_index_2, x_index_2)) || isnan(v(y_index_2, x_index_2))
        cost = 1000000000;
    else
        u_avg_12 = (u(y_index_1, x_index_1) + u(y_index_2, x_index_2)) / 2;
        v_avg_12 = (v(y_index_1, x_index_1) + v(y_index_2, x_index_2)) / 2;

        u_req_12 = (x_2 - x_1) / t_1 - u_avg_12;
        v_req_12 = (y_2 - y_1) / t_1 - v_avg_12;

        vel_req = sqrt(u_req_12^2 + v_req_12^2);

        cost = c_d * vel_req^3 * t_1;
    end
end