% Code for the cost function given a waypoint

function [cost] = cost_function(waypoint)
    x = waypoint(1);
    y = waypoint(2);

    % Simple cost function based on location
    %cost = map(waypoint(1),waypoint(2));
    
    
    %cost = 0;
    if ((x > 0) && (x < 10) && (y > 0) && (y < 10))
        cost = abs(sin(sqrt((x-5)^2+(y-5)^2)) / sqrt((x-5)^2+(y-5)^2));
        %cost = sin(2*x) + sin(2*y) + 3;
    else
        cost = 10;
    end

end