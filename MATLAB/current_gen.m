% This is a script to generate a given current field

%% Simple current field
%   -This current field goes down in the middle and up on the sides

[x,y] = meshgrid(0:.5:10,0:.5:10);

u = zeros(21,21);
v = ones(21,21);

for i = 8:12
    for j = 1:21
        v(j,i) = -1;
    end
end

quiver(x,y,u,v)