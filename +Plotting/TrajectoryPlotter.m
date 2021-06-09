classdef TrajectoryPlotter
    %TrajectoryPlotter Plot trajectory 
    
    %% Private variables
    properties (Access  = private)
        trajectory_line
        trajectory_point
    end
    
    %% Public methods
    methods (Access  = public)
        % Constructor
        function obj = TrajectoryPlotter(AX)
            %TrajectoryPlotter 
            % Initialize title handle
            obj.trajectory_line = plot(AX, NaN, NaN, ...
                'b', 'MarkerSize', 7, 'LineWidth',2.0, 'LineStyle', '-'); 
            obj.trajectory_point = plot(AX, NaN, NaN, ...
                'bx', 'MarkerSize', 7, 'LineWidth',2.0, 'LineStyle', 'None'); 
        end
        
        function updateTrajectoryPoints(obj, x, y)
            %updateTrajectoryPoints Updates trajectory plot - points
            set(obj.trajectory_point, ...
                'XData', x,...
                'YData', y);
        end
        
        function updateTrajectoryLine(obj, x, y)
            %updateTrajectoryLine Updates trajectory plot - line
            set(obj.trajectory_line, ...
                'XData', x,...
                'YData', y);
        end        
    end
end

