% This is a script to generate a given current field

%% Simple current field
%   -This current field goes down in the middle and up on the sides

[q_x,q_y] = meshgrid(0:.5:10,0:.5:10);

v = zeros(21,21);
u = ones(21,21) * -1;

for i = 8:12
    for j = 1:21
        u(i,j) = 1;
    end
end

figure(90)
title('Current Plot')
quiver(q_x,q_y,u,v)