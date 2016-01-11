% Implementation of STOMP Algorithm

%% Initial Setup
clear all
close all
clc

v_max = 2;
num_paths = 50;
num_its = 100;
decay_fact = 1.0;

%% Creating the Map
m_size = 1000;
map = ones(m_size,m_size);

%% Creating the initial path
start_point = [0, 0, 0];
end_point = [10, 10, 0];

num_waypoints = 20;
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

%% Iteration Step

tot_cost = zeros(100,1);
decay_it = decay_fact;

for m = 1:num_its
    
    display(m)
    display(decay_it)
    % Creating the time multiplier matrix

    
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
    
    % Creating the scaling matrix for time update
    scale_T = max(Tinv) * N;
    M_t = zeros(N,N);
    
    for i = 1:N
        M_t(:,i) = Tinv(:,i) / scale_T(i);
    end
    
    % Array for creating the pertubations of the path
    cov_array(:,:,1) = cov_mat;
    cov_array(:,:,2) = cov_mat;
    cov_array(:,:,3) = Tinv;
    
    K = num_paths;

    % Variable to hold all noisy paths generated
    noisy_paths = zeros(K,3,N);
    eps_mat = zeros(K,3,N);

    % Generating the pertubations to the initial path
    for i = 1:K-1
        temp_eps = mvnrnd(zeros(3,N),cov_array) * decay_it;
        temp_eps = temp_eps.';

        temp_eps(1,1) = 0;
        temp_eps(1,2) = 0;

        temp_eps(N,1) = 0;
        temp_eps(N,2) = 0;
        temp_eps(N,3) = 0;

        temp_noisy_path = path(:,:) + temp_eps;
        
        %figure(5)
        %waitforbuttonpress

        eps_mat(i,:,:) = temp_eps.';
        noisy_paths(i,:,:) = temp_noisy_path.';
        % Note: accessing noisy_paths: (path_number, dimension, waypoint_number)
        
        % Looping to ensure that no paths have a negative travel time
        for j = 1:N-1
            if (noisy_paths(i,3,j) <= 0)
                x_dist = noisy_paths(i,1,j) - noisy_paths(i,1,j+1);
                y_dist = noisy_paths(i,2,j) - noisy_paths(i,2,j+1);
                noisy_paths(i,3,j) = 2 * sqrt(x_dist^2 + y_dist^2) / v_max;               
            end
        end
        
        %noisy_paths(i,3,:)
        
        %waitforbuttonpress
     end
    
    temp_eps = zeros(3,N).';
    temp_noisy_path = path(:,:) + temp_eps;
    eps_mat(K,:,:) = temp_eps.';
    noisy_paths(K,:,:) = temp_noisy_path.';

    % Code to plot all the perturbed paths
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
        plot(x,y,'g-x')
    end
    hold off

    cost_mat = zeros(N,K);

    % Loop to calculate all the costs for the noisy paths
    for i = 1:K
        for j = 2:N-1
            waypoint1 = zeros(1,3);
            waypoint2 = zeros(1,3);

            waypoint1(1) = noisy_paths(i,1,j);
            waypoint1(2) = noisy_paths(i,2,j);
            waypoint1(3) = noisy_paths(i,3,j);
            
            waypoint2(1) = noisy_paths(i,1,j);
            waypoint2(2) = noisy_paths(i,2,j);
            waypoint2(3) = noisy_paths(i,3,j);

            cost_mat(j,i) = cost_function_mult_waypoint(waypoint1,waypoint2,v_max);
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

    update_vector = zeros(N,3);

    % Loop through and add all contributions into update vector
    for i = 2:N-1
        for j = 1:K
            update_vector(i,:) = update_vector(i,:) + eps_mat(j,:,i) * prob_mat(i,j);
        end
    end
    
    % Scaling the update vector and ensuring the endpoints do not move
    update_vector(:,1) = M * update_vector(:,1);
    update_vector(:,2) = M * update_vector(:,2);
    update_vector(:,3) = M_t * update_vector(:,3);
    update_vector(1,:) = [0,0,update_vector(1,3)];
    update_vector(N,:) = [0,0,0];
    
    new_path = path(:,:) + update_vector;


    figure(2)
    hold on
    [X,Y] = meshgrid(0:.1:10);
    Z = sin(sqrt((X-5).^2+(Y-5).^2))./sqrt((X-5).^2+(Y-5).^2);
    %Z = sin(2*X) + sin(2*Y) + 3;
    pcolor(X,Y,Z);
    shading flat;
    
    plot(path(:,1),path(:,2),'r',new_path(:,1),new_path(:,2),'g')
    hold off
    
    figure(3)
    hold on
    v_path = cal_velocities(path);
    v_new_path = cal_velocities(new_path);
    plot(1:length(v_path),v_path,'r',1:length(v_new_path),v_new_path,'g')
    hold off
    
    avg_v(m) = mean(v_new_path(2:length(v_new_path)-1));
    avg_v(m)
    
    pause(.1)
    
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
    
    path(:,:) = new_path;
    
    weight = 0.001;
    tot_cost(m) = weight * (.5 * path(:,1).'*R*path(:,1) + .5 * path(:,2).'*R*path(:,2));
   
    for i = 1:N
        tot_cost(m) = tot_cost(m) + cost_function(path(i,1:2),map);
    end
    
    decay_it = decay_it * decay_fact;
    
    
    
end

figure(4)
plot(1:num_its,tot_cost)











