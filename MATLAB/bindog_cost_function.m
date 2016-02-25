% Code for the BinDog cost function
% Calculates the cost for w2 based upon steering angles

function [cost] = bindog_cost_function(w1, w2, w3)
    
    ang_1 = atand((w1(2) - w2(2)) / (w1(1) - w2(1)));
    ang_2 = atand((w2(2) - w3(2)) / (w2(1) - w3(1)));
    
    cost = abs(ang_1 - ang_2);

end