% Function to calculate the velocities along a path with currents

function [velocities] = cal_velocities_curr(path, u, v, size)
    
    N = length(path(:,1));
    velocities = zeros(N-1,1);
    
    for i = 1:N-1
        
        x_1 = path(i,1);
        x_2 = path(i+1,1);
        
        y_1 = path(i,2);
        y_2 = path(i+1,2);
        
        t_1 = path(i,3);
        
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
        
        u_avg_12 = (u(x_index_1, y_index_1) + u(x_index_2, y_index_2)) / 2;
        v_avg_12 = (v(x_index_1, y_index_1) + v(x_index_2, y_index_2)) / 2;
        
        u_req_12 = (x_2 - x_1) / t_1 - u_avg_12;
        v_req_12 = (y_2 - y_1) / t_1 - v_avg_12;
        
        velocities(i) = sqrt(u_req_12^2 + v_req_12^2);
    end
end