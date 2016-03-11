% Script for running tests

%% Run velocity pertubation test

test_mat = [4.76351547943662,4.77697291420761,0.168995041511231;5.12287116129548,5.14830813064594,0.166412721626791];

pert = -.1:.01:.1;

pert_mat = zeros(2,3);

vel = zeros(1,length(pert));

for i = 1:length(pert)
    pert_mat = test_mat;
    pert_mat(1,3) = pert_mat(1,3) + pert(i);
    vel(i) = cal_velocities(pert_mat);
end

plot(1:length(vel),vel)