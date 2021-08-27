%%%%%% Initialize
%%%%%% ini_sensed_points.m
%%%%%%
%%%%%% Initialize sensed graspable points
%%%%%%
%%%%%% Created 2021-02-15
%%%%%% Keigo Haji
%%%%%% Last update: 2021-02-25
%%%%%% Keigo Haji
%%%%%%
%
% Initialize the sensed graspable points based on the initial robot base
% position
%
% Function variables:
%
%     OUTPUT
%        surface_param.sensed_graspable_points :
%               graspable points that have ever been in the camera's FOV
%               3*n matrix
%
%
%     INPUT
%         SV                    : State Variables (SpaceDyn class)
%           RO                  : Initial position of the robot
%         surface_param         : Surface parameters
%           graspable_points    : All the graapable points on the map (3*n)
%         sensing_camera_param  : Sensing Camera parameters
%
%

function[surface_param] = ini_sensed_graspable_points(SV, surface_param, sensing_camera_param)

if strcmp(sensing_camera_param.sensing_flag, 'on')
    % Prepare a matrix that stores the points within the camera range
    [n,m] = size(surface_param.graspable_points);
    surface_param.sensed_graspable_points = NaN(n,m);
    
    if strcmp(sensing_camera_param.known_area_shape, 'rectangle')
        
        % Set the sensed graspable points which the robot has already done
        % sensing before the initial position
        for i = 1:length(surface_param.graspable_points)
            % For the x-axis
            if surface_param.graspable_points(1,i) < SV.R0(1)+sensing_camera_param.ini_margin_from_base_pos_to_plus_x
                % For the y-axis
                if (surface_param.graspable_points(2,i) < SV.R0(2) +sensing_camera_param.ini_margin_from_base_pos_to_y) && (surface_param.graspable_points(2,i) > SV.R0(2) - sensing_camera_param.ini_margin_from_base_pos_to_y)
                    % Add the point positons to the matrix.
                    surface_param.sensed_graspable_points(:,i) = surface_param.graspable_points(:,i);
                end
            end
        end
    end
    if strcmp(sensing_camera_param.known_area_shape, 'circle')
        % Convert the coordinates of graspable points to coordinates from the base center.
        graspable_points_pos_from_base_pos = surface_param.graspable_points - SV.R0;
        for i = 1:length(surface_param.graspable_points)
            % Calculate the distance from the base pos to each graspable
            % point
            distance_from_base_pos = sqrt(graspable_points_pos_from_base_pos(1,i)^2 + graspable_points_pos_from_base_pos(2,i)^2);
            if distance_from_base_pos < sensing_camera_param.ini_circular_radius_from_base_pos
                surface_param.sensed_graspable_points(:,i) = surface_param.graspable_points(:,i);
            end
        end
    end
    
end

end

