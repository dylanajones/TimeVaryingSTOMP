% STOMP with Real Currents
%   -Current data is taken from ROMS data set

%% Initial Setup
clear all
close all
clc

v_max = 2;
num_paths = 20;
num_its = 100;
decay_fact = .99;

%% Creating the Current Map

filename = 'temp3.nc';

ncid = netcdf.open(filename);

varname = netcdf.inqVar(ncid,2);
varid = netcdf.inqVarID(ncid,varname);

lat = netcdf.getVar(ncid,varid);

varname = netcdf.inqVar(ncid,3);
varid = netcdf.inqVarID(ncid,varname);

lon = netcdf.getVar(ncid,varid);

q_x = zeros(length(lat),length(lon));
q_y = zeros(length(lat),length(lon));

for i = 1:length(lon)
    q_y(:,i) = lat;
end

for i = 1:length(lat)
    q_x(i,:) = lon;
end

varname = netcdf.inqVar(ncid,6);
varid = netcdf.inqVarID(ncid,varname);

data = netcdf.getVar(ncid,varid);

u = data(:,:,1);
u = u.';

varname = netcdf.inqVar(ncid,7);
varid = netcdf.inqVarID(ncid,varname);

data = netcdf.getVar(ncid,varid);

v = data(:,:,1);
v = v.';

[n,m] = size(v);

mag = zeros(size(v));

for i = 1:n
    for j = 1:m
        flag = false;
        
        if u(i,j) == -9999
            u(i,j) = NaN;
            flag = true;
        end
        
        if v(i,j) == -9999
            v(i,j) = NaN;
            flag = true;
        end
        
        if flag == false
            mag(i,j) = sqrt(u(i,j)^2 + v(i,j)^2);
        else
            mag(i,j) = NaN;
        end
    end
end


figure 
contourf(q_x(30:50,70:100),q_y(30:50,70:100),mag(30:50,70:100),'LineColor','none');
caxis([0,max(max(mag))]); colormap (jet); 
colorbar 
hold on;

idx = ~isnan(u) & ~isnan(v);

quiver(q_x(idx),q_y(idx),u(idx),v(idx),'LineWidth',1,'Color','k');

%% Creating the initial path

% making a start and goal - coordinates in latitude and longitude
start_point = [33.3, 240.3, 0];
end_point = [33.2, 240.6, 0];

% Setting the number of waypoints including the start and goal
num_waypoints = 50;
N = num_waypoints;

% Initializing the path variable
path = zeros(num_waypoints,3);
path(1,:) = start_point;
path(num_waypoints,:) = end_point;

% Calculating the stright line path step size
x_step = abs(path(1,1)-path(num_waypoints,1)) / (num_waypoints - 1);
y_step = abs(path(1,2)-path(num_waypoints,2)) / (num_waypoints - 1);

% Initializing the t value to a reasonable value
t_init = sqrt(x_step^2 + y_step^2) / v_max;

path(:,3) = t_init;
path(num_waypoints,3) = 0;

% Intializing all the waypoints in the path
for i = 2:num_waypoints-1
    path(i,1) = path(i-1,1) + x_step;
    path(i,2) = path(i-1,2) + y_step;
end









































