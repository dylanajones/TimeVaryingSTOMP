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
    cost = 0;
    if ((x > 4) && (x < 6) && (y > 4) && (y < 6))
        cost = 1 / sqrt((x-5)^2+(y-5)^2);
    end
end