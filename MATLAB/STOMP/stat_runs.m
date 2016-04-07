num_runs = 100;

energy_cost_mat = zeros(100,num_runs);

for p = 1:num_runs
    main
    display(energy_cost);
    energy_cost_mat(:,p) = energy_cost();
    display('This is run')
    display(p)
    pause(.5)
    %figure(50)
    %waitforbuttonpress()
end