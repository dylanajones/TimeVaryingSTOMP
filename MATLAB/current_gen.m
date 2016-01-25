% This is a script to generate a given current field

function [u,v,q_x,q_y] = current_gen(flag)

% Simple current field
%   -This current field goes down in the middle and up on the sides

    if flag == 1

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

% Circular Current Field
%   -This current field is circular and stronger as you go outward

    elseif flag == 2

        [q_x,q_y] = meshgrid(0:.5:10,0:.5:10);

        v = zeros(21,21);
        u = zeros(21,21);

        for i = 1:length(q_x)
            for j = 1:length(q_y)
                u(i,j) = q_y(i,j)-5;
                v(i,j) = -q_x(i,j)+5;
            end
        end
        
        %u = u / 5;
        %v = v / 5;
        
        figure(90)
        title('Current Plot')
        quiver(q_x,q_y,u,v)



% Double Circle Current Field
    elseif flag == 3
        [q_x,q_y] = meshgrid(0:.5:10,0:.5:10);

        v = zeros(21,21);
        u = zeros(21,21);

        for i = 1:length(q_x)
            for j = 1:length(q_y) - i + 1
                u(i,j) = u(i,j) + q_y(i,j);
                v(i,j) = v(i,j) + -q_x(i,j);
            end
        end

        for i = 1:length(q_x)
            for j = length(q_y)-i+1:length(q_y)
                u(i,j) = u(i,j) + q_y(i,j)-10;
                v(i,j) = v(i,j) + -q_x(i,j)+10;
            end
        end

        figure(90)
        title('Current Plot')
        quiver(q_x,q_y,u,v)
    end
end

%% Test Current field
% 
% [q_x,q_y] = meshgrid(0:.5:10,0:.5:10);
% 
% v = zeros(21,21);
% u = zeros(21,21);
% 
% for i = 1:length(q_x)
%     for j = 1:length(q_y)
%         u(i,j) = q_y(i,j);
%         v(i,j) = -q_x(i,j);
%     end
% end
% 
% for i = 1:length(q_x)
%     for j = 1:length(q_y)
%         u(i,j) = u(i,j) + q_y(i,j)-10;
%         v(i,j) = v(i,j) + -q_x(i,j)+10;
%     end
% end
% 
% figure(90)
% title('Current Plot')
% quiver(q_x,q_y,u,v)

















