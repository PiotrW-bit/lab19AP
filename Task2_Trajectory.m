%% Clear workspace before run
clear all
close all

%% USER INPUT: Path to file to load
% Please change only code in the highlighted areas for simpler error tracking
grid_data_path = 'test_data/occupancy_grid_recording.mat';
host_data_path = 'test_data/host_data.mat';
scans_to_plot = []; % set to empty to plot all scans

%% Load grid file and convert to correct format
load(grid_data_path);
grid_map = Utilities.loadGridMap(grid_data); % Uncompress the mat_file data

% Create scans to scans_to_plot
if(isempty(scans_to_plot))
    scans_to_plot = 1:scan_number;
else
    scans_to_plot(scans_to_plot>scan_number) = [];
    scans_to_plot(scans_to_plot<1) = [];
end

%% Load Host data
host = load(host_data_path);

%% Initialize Grid Plotter
fig = figure();
AX = axes(fig);
Plotting.initializePlotter(AX, grid_size);
grid_plotter = Plotting.GridPlotter(AX, grid_size, grid_resolution);
host_plotter = Plotting.HostPlotter(AX);
trajectory_plotter = Plotting.TrajectoryPlotter(AX);

%% Run Scenario
% Default variable values - remeber to fill them during exectution
time_to_collision = 0;
trajectory_x = [];
trajectory_y = [];
time_to_collision_per_scan = zeros(1,length(scans_to_plot));
time = zeros(1,length(scans_to_plot));
for idx = 1:length(scans_to_plot)
    % Load file and data relevant for algorithm
    scan_idx = scans_to_plot(idx);
    grid_map_current = grid_map{scan_idx};
    host_pose_current = host_pose{scan_idx};
    
    orientation = -host.host_x_RPY{scan_idx}.y;
    linear_velocity = sqrt(host.host_v_XYZ{scan_idx}.x.^2 + host.host_v_XYZ{scan_idx}.y.^2);
    linear_acceleration = sqrt(host.host_a_XYZ{scan_idx}.x.^2 + host.host_a_XYZ{scan_idx}.y.^2);
    angular_velocity = -host.host_v_RPY{scan_idx}.y;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% Insert Trajectory code %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    estimation_time = 0:0.01:5;
    % Calculate trajectory offsets
	[ x_offset , y_offset , o ] = CACTR ( estimation_time , orientation , linear_velocity , linear_acceleration , angular_velocity ) ;

	% Calculate trajectory points
	trajectory_x = host_pose_current . x_position + x_offset ;
	trajectory_y = host_pose_current . y_position + y_offset ;
    
    % Remove points outside of the grid
	invalid_trajectory_points = ( trajectory_x <0) |( trajectory_x > grid_size ) | ( trajectory_y <0) |( trajectory_y > grid_size) ;
	estimation_time ( invalid_trajectory_points ) = [];
	trajectory_x ( invalid_trajectory_points ) = [];
	trajectory_y ( invalid_trajectory_points ) = [];

    % Convert points to grid cell subscripts
	[ col , row ] = Utilities . OGCS2Subscripts ( trajectory_x , trajectory_y , grid_resolution) ;
    grid_cell_idx = col + ( row -1) * size ( grid_map_current ,1) ;
    
    % Find first occupied cell -> end of trajectory
	cells_on_trajectory = grid_map_current ( grid_cell_idx ) ;
	first_occupied_cell_on_trajectory_idx = find (cells_on_trajectory > 0.8 , 1, 'first');
    
    % Compute time to collision
	time_to_collision = max ( estimation_time ) ;
	if (~ isempty ( first_occupied_cell_on_trajectory_idx ) )
        time_to_collision = estimation_time (first_occupied_cell_on_trajectory_idx) ;
	end
    
    % Remove trajectory points after detected object
	if (~ isempty ( first_occupied_cell_on_trajectory_idx ) )
        trajectory_x (( first_occupied_cell_on_trajectory_idx +1) : end ) = [];
        trajectory_y (( first_occupied_cell_on_trajectory_idx +1) : end ) = [];
	end

    % Debug message - can be commented out
    fprintf("Time to colision for idx %d is equal to: %2.2fs \n", scan_idx, time_to_collision);
    
    
    
    % Accumulate time_to_collisions for further analisis
    time_to_collision_per_scan(idx) = time_to_collision;
    time(idx) = host_pose_current.ts;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%  Plotter section  %%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grid_plotter.updateGrid(grid_map_current, ...
        sprintf("Scan nr: %d/%d, time from beginning: %2.2fs", scan_idx, scan_number, host_pose_current.ts));
    host_plotter.updateHostPose(host_pose_current);
    trajectory_plotter.updateTrajectoryLine(trajectory_x, trajectory_y);
    pause(0.01); % required to force figure update
end

%% Further analisis section
b = 1;
a = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
time_to_collision_per_scan_filtered = filter(a,b,time_to_collision_per_scan);
figure()
plot(time, time_to_collision_per_scan_filtered)
grid on;
title("Wykres czasu do kolizji")
xlabel("Czas [s]")
ylabel("Czas do kolizji")
%% Function
function [x_offset, y_offset, o] = CACTR(prediction_time, o, v, a, v_o)
% Estimate host_data to new view using CTRV model
% doi:10.3390/s140305239
% o - orientation
% v - linear_velocity
% a - linear_acceleration
% v_o - angular_velocity
    % Constant Turn Rate Constant Acceleration Model
    d_c = prediction_time.*(v + a.*prediction_time); % distance passed
    
    o_end = o + v_o .* prediction_time;
    o_begin = o;
    y_offset =  d_c .* (cos(o_end) - cos(o_begin))./(o_end - o_begin);
    x_offset =  d_c .* (sin(o_end) - sin(o_begin))./(o_end - o_begin);

    % Manage no rate change - Constant Acceleration Model
    x_offset(isnan(x_offset)) = d_c(isnan(x_offset))*sin(o);
    y_offset(isnan(y_offset)) = d_c(isnan(y_offset))*cos(o);
    
    o = o_end;
end