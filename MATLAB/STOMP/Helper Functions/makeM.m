% This is a function to make the M matrix

function [M] = makeM(R)

    len = length(R);
    M = zeros(len,len);
    
    for j = 1:len
        maxC = max(R(:,j));
        for k = 1:len
            M(k,j) = R(k,j) / (maxC * len);
        end
    end
        

end