classdef ContourPlotter
    %ContourPlotter Plotter class for plotting contours
    
    %% Private variables
    properties (Access  = private)
        contour_handles
        resolution
    end
    
    %% Public methods
    methods (Access  = public)
        % Constructor
        function obj = ContourPlotter(AX, resolution, max_nr_of_contours)
            %ContourPlotter Create handles for plotting contours on
            %provided axes handle
            % Input: 
            % AX - axes handle
            % resolution - grid scaling factor [m]
            % max_nr_of_contours - supported number of contours  
            obj.resolution = resolution;
            obj.contour_handles = cell(1,max_nr_of_contours);
            for idx = 1:max_nr_of_contours
                obj.contour_handles{idx} = patch(AX, 'XData',[],'YData',[],'EdgeColor','cyan',...
                'FaceColor','none','LineWidth',2);
            end
        end
        
        function updateContours(obj, contours_cell_array)
            %updateContours Updates contour patches
            % Input:
            % contours_cell_array - cell array with 
            if(length(contours_cell_array) > length(obj.contour_handles))
                warning("Number of contours bigger than supported. Skipping extra contours");
            end
            
            for idx=1:length(obj.contour_handles)
                if(idx<length(contours_cell_array))
                    y_data = (contours_cell_array{idx}(:,1)-1/2)*obj.resolution; 
                    x_data = (contours_cell_array{idx}(:,2)-1/2)*obj.resolution;
                else
                    x_data = [];
                    y_data = [];
                end
                set(obj.contour_handles{idx}, ...
                    'XData', x_data, ...
                    'YData', y_data);
            end         
        end
    end
end

