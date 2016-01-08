% Code for the cost function given a waypoint

% TODO:
%   -Will need to write actual cost function, also needs to take in more
%   waypoints

function [cost] = cost_function(waypoint, map)
    x = waypoint(1);
    y = waypoint(2);

    % Simple cost function based on location
    %cost = map(waypoint(1),waypoint(2));
    
    %Block in middle cost function
    %cost = 0;
    if ((x > 0) && (x < 10) && (y > 0) && (y < 10))
        %cost = sin(sqrt((x-5)^2+(y-5)^2)) / sqrt((x-5)^2+(y-5)^2);
        cost = sin(x) + sin(y) + 2;
    else
        cost = 10;
    end

end