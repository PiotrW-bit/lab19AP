classdef HostPlotter
    %HostPlotter Plotter class for plotting host rectangle
    
    %% Private variables
    properties (Access  = private)
        host_bounding_box
        host_front_arrow
        host_point
        host_parameters % struct
        host_points % constant rectangle points
    end
    
    %% Public methods
    methods (Access  = public)
        % Constructor
        function obj = HostPlotter(AX)
            %HostPlotter Initialize all plot handles
            %used in plotter
            obj.host_bounding_box = patch(AX, 'XData',zeros(1,4),'YData',zeros(1,4),'EdgeColor','magenta',...
                'FaceColor','none','LineWidth',2);  
            obj.host_front_arrow = patch(AX, 'XData',zeros(1,4),'YData',zeros(1,4),'EdgeColor','green',...
                'FaceColor','none','LineWidth',2);
            obj.host_point = plot(AX, NaN, NaN, ...
                'mx', 'MarkerSize', 7,'LineWidth',1.0);  
            
            obj = initializeHostParameters(obj);            
        end
        
        function updateHostPose(obj, host_pose)
            %UPDATEHOSTPOSE Updates host pose (x mark, bounding box and
            %triangle)
            %% Host X - center rear axle
            set(obj.host_point, ...
                'XData', host_pose.x_position, ...
                'YData', host_pose.y_position);
            
            %% Host rectangle
            corners_x_2d =   obj.host_points.bb_y .* (ones(5, 1) * cos(host_pose.orientation)) - obj.host_points.bb_x .* (ones(5, 1) * sin(host_pose.orientation));
            corners_y_2d = + obj.host_points.bb_y .* (ones(5, 1) * sin(host_pose.orientation)) + obj.host_points.bb_x .* (ones(5, 1) * cos(host_pose.orientation));
            
            corners_x_2d = corners_x_2d + host_pose.x_position;
            corners_y_2d = corners_y_2d + host_pose.y_position;
                        
            set(obj.host_bounding_box, ...
                'XData', corners_x_2d, ...
                'YData', corners_y_2d); 
            
            % host front triangle
            corners_x_front =   obj.host_points.triangle_y .* (ones(4, 1) * cos(host_pose.orientation)) - obj.host_points.triangle_x .* (ones(4, 1) * sin(host_pose.orientation));
            corners_y_front = + obj.host_points.triangle_y .* (ones(4, 1) * sin(host_pose.orientation)) + obj.host_points.triangle_x .* (ones(4, 1) * cos(host_pose.orientation));
            
            corners_x_front = corners_x_front + host_pose.x_position;
            corners_y_front = corners_y_front + host_pose.y_position;
            
            set(obj.host_front_arrow, ...
                'XData', corners_x_front, ...
                'YData', corners_y_front);
        end
    end
    
    %% Private methods
    methods (Access  = private)
        function obj = initializeHostParameters(obj)
            obj.host_parameters.length     = 5;
            obj.host_parameters.width      = 2;
            obj.host_parameters.rear_overhang   = 1; % host rear bumber relative to host_pose
            obj.host_parameters.front  = 4; % host front bumper relative to host_pose
            obj.host_parameters.front_axle  = 3;  % host front axle relative to host_pose    
            
            % Host points
            obj.host_points.bb_y = [ -obj.host_parameters.rear_overhang; obj.host_parameters.front; obj.host_parameters.front;  -obj.host_parameters.rear_overhang; -obj.host_parameters.rear_overhang];
            obj.host_points.bb_x = [  obj.host_parameters.width/2;  obj.host_parameters.width/2;  -obj.host_parameters.width/2;    -obj.host_parameters.width/2;   obj.host_parameters.width/2];
                        
            obj.host_points.triangle_y = [  obj.host_parameters.front; obj.host_parameters.front_axle; obj.host_parameters.front_axle;  obj.host_parameters.front];
            obj.host_points.triangle_x = [  0;          obj.host_parameters.width/2;       -obj.host_parameters.width/2;             0     ];
        end
    end
end

