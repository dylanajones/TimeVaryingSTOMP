% Implementation of STOMP Algorithm

%% Initial Setup
clear
close all
clc

% Velocity in meters / second
v_max = 1;

% Parameters
num_paths = 100;
num_its = 20;

%% Creating the initial path
start_point = [1, 10, 0];
center_point = [10, 10, 0];
end_point = [10, 1, 0];

num_waypoints = 30;
N = num_waypoints;

path = zeros(num_waypoints,3);
path(1,:) = start_point;
path(num_waypoints,:) = end_point;
path(num_waypoints/2,:) = center_point;

x_step_1 = (center_point(1) - path(1,1)) / (num_waypoints / 2 - 1);
y_step_1 = (center_point(2) - path(1,2)) / (num_waypoints / 2 - 1);

x_step_2 = (path(num_waypoints,1) - center_point(1)) / (num_waypoints / 2 - 1);
y_step_2 = (path(num_waypoints,2) - center_point(2)) / (num_waypoints / 2 - 1);

t_init_1 = sqrt(x_step_1^2 + y_step_1^2) / v_max;
t_init_2 = sqrt(x_step_2^2 + y_step_2^2) / v_max;

path(1:num_waypoints/2,3) = t_init_1;
path(num_waypoints/2:num_waypoints,3) = t_init_1;
path(num_waypoints,3) = 0;

for i = 2:num_waypoints/2-1
    path(i,1) = path(i-1,1) + x_step_1;
    path(i,2) = path(i-1,2) + y_step_1;
end

for i = num_waypoints/2 + 1:num_waypoints-1
    path(i,1) = path(i-1,1) + x_step_2;
    path(i,2) = path(i-1,2) + y_step_2;
end

plot(path(:,1),path(:,2),'-x')

%% Starting Calculations

% Use center finite diff creation
B = finitediff2(N);

Rinv = inv(B);
RinvTran = inv(B.');

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

% Creating R matrix
R = B.'* T * T * B;

% Creating the covariance matrix from which the pertubations are sampled
cov_mat = Rinv * Tinv * Tinv * RinvTran;

% This is needed due to numerical errors to ensure the mvnrnd function can
% be used
cov_mat = (cov_mat + cov_mat.') / 2;

% Creating the scaling matrix for update
scale_M = max(cov_mat) * N;
M = zeros(N,N);

for i = 1:N
    M(:,i) = cov_mat(:,i) / scale_M(i);
end

cov_array(:,:,1) = cov_mat;
cov_array(:,:,2) = cov_mat;

%% Iteration Step

tot_cost = zeros(num_its,1);
K = num_paths;

mag = .15;

for m = 1:num_its
    display(m)

    % Variable to hold all noisy paths generated
    noisy_paths = zeros(K,2,N);
    eps_mat = zeros(K,2,N);

    % Generating the pertubations to the initial path
    for i = 1:K
        temp_eps = mvnrnd(zeros(2,N),cov_array) * mag;
        temp_eps = temp_eps.';
        
%         display(temp_eps)
%         waitforbuttonpress
        
        temp_eps(1,1) = 0;
        temp_eps(1,2) = 0;

        temp_eps(N,1) = 0;
        temp_eps(N,2) = 0;

        temp_noisy_path = path(:,1:2) + temp_eps;

        eps_mat(i,1:2,:) = temp_eps.';
        noisy_paths(i,1:2,:) = temp_noisy_path.';
        % Note: accessing noisy_paths: (path_number, dimension, waypoint_number)
    end

%     % Code to plot all the perturbed paths
    figure(1)
    clf
    hold on
    plot(path(:,1),path(:,2),'r-x')

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
    
    pause(.5)
    
    cost_mat = zeros(N,K);

    % Loop to calculate all the costs for the noisy paths
    for i = 1:K
        for j = 2:N-1
            waypoint_1 = zeros(1,2);
            waypoint_2 = zeros(1,2);
            waypoint_3 = zeros(1,2);

            waypoint_1(1) = noisy_paths(i,1,j-1);
            waypoint_1(2) = noisy_paths(i,2,j-1);
            
            waypoint_2(1) = noisy_paths(i,1,j);
            waypoint_2(2) = noisy_paths(i,2,j);
            
            waypoint_3(1) = noisy_paths(i,1,j+1);
            waypoint_3(2) = noisy_paths(i,2,j+1);

            cost_mat(j,i) = bindog_cost_function(waypoint_1, waypoint_2, waypoint_3);
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
    for i = 2:N-1
        e_sum = 0;

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
    for i = 2:N-1
        for j = 1:K
            update_vector(i,:) = update_vector(i,:) + eps_mat(j,:,i) * prob_mat(i,j);
        end
    end
    
    % Scaling the update vector and ensuring the endpoints do not move
    update_vector(:,1) = M * update_vector(:,1);
    update_vector(:,2) = M * update_vector(:,2);
    update_vector(1,:) = [0,0];
    update_vector(N,:) = [0,0];
    
    new_path = path(:,1:2) + update_vector;


%     figure(2)
%     hold on
%     [X,Y] = meshgrid(-1:.5:15);
%     %Z = sin(sqrt((X-5).^2+(Y-5).^2))./sqrt((X-5).^2+(Y-5).^2);
%     Z = sin(X) + cos(Y);
%     pcolor(X,Y,Z);
%     shading flat;
%     
%     plot(path(:,1),path(:,2),'r',new_path(:,1),new_path(:,2),'g')
%     hold off
%     
%     pause(.01)

    path(:,1:2) = new_path;
    
    weight = 0.0001;
    tot_cost(m) = weight * (.5 * path(:,1).'*R*path(:,1) + .5 * path(:,2).'*R*path(:,2));
    
    for i = 2:N-1    
        
        tot_cost(m) = tot_cost(m) + bindog_cost_function(path(i-1,1:2),path(i,1:2),path(i+1,1:2));
    end
    
    %display(tot_cost)
end

% figure
% hold on
% [X,Y] = meshgrid(0:.1:15);
% %Z = abs(sin(sqrt((X-5).^2+(Y-5).^2))./sqrt((X-5).^2+(Y-5).^2));
% Z = sin(X) + sin(Y);
% pcolor(X,Y,Z);
% shading flat;
% 
% plot(path(:,1),path(:,2),'r',new_path(:,1),new_path(:,2),'g')
% hold off

figure
plot(1:num_its,tot_cost)

figure
plot(path(:,1),path(:,2),'r-x')