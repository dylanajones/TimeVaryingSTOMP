% Implementation of STOMP Algorithm

%% Initial Setup
clear all
close all
clc

v_max = 2;
num_paths = 10;

%% Creating the Map
m_size = 1000;
map = ones(m_size,m_size);

%% Creating the initial path
start_point = [0, 0, 0];
end_point = [10, 10, 0];

num_waypoints = 100;
N = num_waypoints;

path = zeros(num_waypoints,3);
path(1,:) = start_point;
path(num_waypoints,:) = end_point;

x_step = abs(path(1,1)-path(num_waypoints,1)) / (num_waypoints - 1);
y_step = abs(path(1,2)-path(num_waypoints,2)) / (num_waypoints - 1);

t_init = sqrt(x_step^2 + y_step^2) / v_max;

path(:,3) = t_init;
path(num_waypoints,3) = 0;

for i = 2:num_waypoints-1
    path(i,1) = path(i-1,1) + x_step;
    path(i,2) = path(i-1,2) + y_step;
end

%% Starting Calculations

% Use balanced finite diff creation
%B = finitediff(N);

% Use center finite diff creation
B = finitediff2(N);

Rinv = inv(B);
RinvTran = inv(B.');

%% Iteration Step

for m = 1:50
    display(m)
    % Creating the time multiplier matrix
    T = eye(N);
    Tinv = T;

    % Handling the two end cases
    T(1,1) = 1 / (path(1,3) ^ 2);
    Tinv(1,1) = path(1,3) ^ 2;
    T(N,N) = 1 / (path(N-1,3) ^ 2);
    Tinv(N,N) = path(N-1,3) ^ 2;

    % Taking the square of the average time 
    for i = 2:N-1
        T(i,i) = 1 / (((path(i-1,3) + path(i,3)) / 2) ^ 2);
        Tinv(i,i) = (((path(i-1,3) + path(i,3)) / 2) ^ 2);
    end

    % Creating the covariance matrix from which the pertubations are sampled
    cov_mat = Rinv * Tinv * Tinv * RinvTran;

    % This is needed due to numerical errors to ensure the mvnrnd function can
    % be used
    cov_mat = (cov_mat + cov_mat.') / 2;

    cov_array(:,:,1) = cov_mat;
    cov_array(:,:,2) = cov_mat;

    K = num_paths;

    % Variable to hold all noisy paths generated
    noisy_paths = zeros(K,2,N);
    eps_mat = zeros(K,2,N);

    % Generating the pertubations to the initial path
    for i = 1:K
        temp_eps = mvnrnd(zeros(2,N),cov_array);
        temp_eps = temp_eps.';

        temp_eps(1,1) = 0;
        temp_eps(1,2) = 0;

        temp_eps(N,1) = 0;
        temp_eps(N,2) = 0;

        temp_noisy_path = path(:,1:2) + temp_eps;

        eps_mat(i,1:2,:) = temp_eps.';
        noisy_paths(i,1:2,:) = temp_noisy_path.';
        % Note: accessing noisy_paths: (path_number, dimension, waypoint_number)
    end

    % Code to plot all the perturbed paths
    figure(1)
    clf
    hold on
    plot(path(:,1),path(:,2),'r')

    x = zeros(N,1);
    y = x;

    for i = 1:K
        for j = 1:N
            x(j) = noisy_paths(i,1,j);
            y(j) = noisy_paths(i,2,j);
        end
        plot(x,y,'g')
    end
    hold off

    cost_mat = zeros(N,K);

    % Loop to calculate all the costs for the noisy paths
    for i = 1:K
        for j = 1:N
            waypoint = zeros(1,2);

            waypoint(1) = noisy_paths(i,1,j);
            waypoint(2) = noisy_paths(i,2,j);

            cost_mat(j,i) = cost_function(waypoint,map);
        end   
    end

    % Finding the minimum and maximum costs for each watypoint number
    max_costs = max(cost_mat.');
    min_costs = min(cost_mat.');

    % Finding the probability contribution of each waypoint

    prob_mat = zeros(N,K);
    e_mat = zeros(N,K);

    h = 10;

    % For this loop we go through the waypoints on the outside and the paths on
    % the inside as we calculate the probability
    for i = 1:N
        e_sum = 0;

        %TODO: NEED TO ADD CODE TO ACCOUNT FOR MIN AND MAX COST BEING EQUAL
        if max_costs(i) ~= min_costs(i)
            for j = 1:K
                e_mat(i,j) = exp(-h*((cost_mat(i,j) - min_costs(i))/(max_costs(i) - min_costs(i))));
                e_sum = e_sum + e_mat(i,j);
            end

            prob_mat(i,:) = e_mat(i,:) / e_sum;
        else
            prob_mat(i,:) = 1 / K;
        end
    end

    update_vector = zeros(N,2);

    % Loop through and add all contributions into update vector
    for i = 1:N
        for j = 1:K
            update_vector(i,:) = update_vector(i,:) + eps_mat(j,:,i) * prob_mat(i,j);
        end
    end

    % TODO: NEED TO SCALE THE UPDTAE VECTOR HERE


    new_path = path(:,1:2) + update_vector;


    figure(2)
    plot(path(:,1),path(:,2),'r',new_path(:,1),new_path(:,2),'b')
    rectangle('Position',[4,4,2,2])

    w = waitforbuttonpress;

    path(:,1:2) = new_path;

end













