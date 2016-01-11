% This is a function to calculate direction of Vmax
%   -Returns a unit vector in the direction that Vmax should be directed

function [x,y] = v_max_direction(w1, w2, v_max, c_vec)
    
    c_unit = c_vec / norm(c_vec);
    
    d = w2 - w1

end