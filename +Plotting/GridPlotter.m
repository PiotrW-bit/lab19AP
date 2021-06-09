classdef GridPlotter
    %GRIDPLOTTER Plotter class for plotting occupancy grid
    
    %% Private variables
    properties (Access  = private)
        grid_image_handle
        title_handle
    end
    
    %% Public methods
    methods (Access  = public)
        % Constructor
        function obj = GridPlotter(AX, grid_size, grid_resolution)
            %GRIDPLOTTER Constructs figure and initialize plot handles
            %for grid and title
            % Initialize title handle
            obj.title_handle = title(AX, "Empty GridPlotter", 'Interpreter', 'None');
            
            % initialize grid image as empty
            grid_size_px = round(grid_size/grid_resolution);
            grid_cell_centers = -grid_resolution/2 + (1:grid_size_px)*grid_resolution;
            obj.grid_image_handle = imagesc(AX, 'XData', grid_cell_centers, 'YData', grid_cell_centers, 'CData', zeros(grid_size_px, grid_size_px));
        end
        
        function updateGrid(obj, grid_map, title_string)
            %UPDATEGRID Updates grid image and title
            set(obj.grid_image_handle, ...
                'CData', grid_map');
            set(obj.title_handle, ...
                'String', title_string);            
        end
    end
end

