function [grid_map] = loadGridMap(grid_data)
% For better compression probabilities are sroted as uin8 values in range
% from 0-100
% This is very stupid compression technique, but the data are easy to
% understand. In real algorithm the log-it transofmration is used to
% convert probability into uint8 (read it on Wiki :) )

grid_map = cell(size(grid_data));

for scan_idx = 1:length(grid_data)
    grid_map{scan_idx} = double(grid_data{scan_idx})/100;
end
end

