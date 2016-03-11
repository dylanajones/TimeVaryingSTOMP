function [ h ] = hueristic1( node1, node2, v_max )
%HUERISTIC Calculates the hueristic cost betwewen two nodes
%   Detailed explanation goes here
    
    cd = 3;
    
    x_dist = node2(1) - node1(1);
    y_dist = node2(2) - node1(2);
    
    tot_dist = sqrt(x_dist^2 + y_dist^2);
    
    t_req = tot_dist / v_max;
    
    h = cd * v_max^3 * t_req;

end

