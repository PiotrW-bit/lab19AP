%% Clear workspace before run
clear all
close all

%% USER INPUT: Path to file to load
% Please change only code in the highlighted areas for simpler error tracking
grid_data_path = 'test_data\occupancy_grid_recording.mat';
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

%% Initialize Grid Plotter
fig = figure();
AX = axes(fig);
Plotting.initializePlotter(AX, grid_size);
grid_plotter = Plotting.GridPlotter(AX, grid_size, grid_resolution);
host_plotter = Plotting.HostPlotter(AX);
contour_plotter = Plotting.ContourPlotter(AX, grid_resolution, 100);

%% Run Scenario
for scan_idx = scans_to_plot
    grid_map_current = grid_map{scan_idx};
    host_pose_current = host_pose{scan_idx};
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%% Insert Contour Code here %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bin = grid_map_current > 0.53;
    se = strel('disk', 4);
    bin_closed = imclose(bin, se);
    % Filter image based on image properties.
    bin_closed = bwpropfilt(bin_closed, 'Area', [40, Inf]);

    % Get properties.
    properties = regionprops(bin_closed, 'Area', 'ConvexHull'); 
    % Tip: Use Matlab tool image region analizer
    
    
    % Prepare data for plotting
    % contours - this variable should be cell array with each cell containing Nx2
    % vectors of X Y contour point coordinates i.e. contours={properties.ConvexHull}
    % grid_map_to_display - this variable will be visualized. Change it to debug image processign steps
    contours = {properties.ConvexHull};
    grid_map_to_display = grid_map_current; % this variable will be visualized. Change it to debug image processign steps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%  Plotter section  %%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    grid_plotter.updateGrid(grid_map_to_display, ...
        sprintf("Scan nr: %d/%d, time from beginning: %2.2fs", ...
        scan_idx, scan_number, host_pose_current.ts));
    host_plotter.updateHostPose(host_pose_current);
    contour_plotter.updateContours(contours);
    
    pause(0.01); % required to force figure update
end