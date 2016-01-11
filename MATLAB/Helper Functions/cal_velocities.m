% Function to calculate the velocities along a path

function [velocities] = cal_velocities(path)
    
    N = length(path(:,1));
    velocities = zeros(N-1,1);
    
    for i = 1:N-1
        x_1 = path(i,1);
        x_2 = path(i+1,1);
        
        y_1 = path(i,2);
        y_2 = path(i+1,2);
        
        t = path(i,3);
        
        velocities(i) = sqrt((x_1 - x_2)^2 + (y_1 - y_2)^2) / t;
    end
end