close all
clear
clc

num_runs = 100;

energy_cost_mat = zeros(100,num_runs);
time_mat = zeros(1,num_runs);

for p = 1:num_runs
    %main
    main_with_real_currents
    %main_adv_working_v_no_t_varying
    display(energy_cost);
    energy_cost_mat(1:length(energy_cost),p) = energy_cost();
    time_mat(p) = time;
    path_mat(p) = {path};
    display('This is run')
    display(p)
    pause(.5)
    %figure(50)
    %waitforbuttonpress()
end

for i = 1:100
    x = find(energy_cost_mat(:,i)==0);
    if isempty(x)
        energy_final(i) = energy_cost_mat(100,i);
    else
        energy_final(i) = energy_cost_mat(x(1)-1,i);
    end
end