%%%%%% Update
%%%%%% upd_sensed_points.m
%%%%%%
%%%%%% Update sensed graspable points
%%%%%%
%%%%%% Created 2021-02-15
%%%%%% Keigo Haji
%%%%%% Last update: 2021-02-25
%%%%%% Keigo Haji
%%%%%%
%
% Update sensed_graspable_points with the points that entered the camera's
% angle of view
%
%
% Function variables:
%
%     OUTPUT
%        surface_param.sensed_graspable_points :
%               graspable points that have ever been in the camera's FOV
%               3*n matrix
%
%     INPUT
%         SV                    : State Variables (SpaceDyn class)
%         surface_param         : Surface parameters
%           graspable_points    : All the graapable points on the map (3*n)
%         sensing_camera_param  : Sensing Camera parameters
%
%

function [ surface_param ] = upd_sensed_graspable_points(SV, surface_param, sensing_camera_param)

if strcmp(sensing_camera_param.sensing_flag, 'on')
    if strcmp(sensing_camera_param.sensing_type, 'RealSense_d435i')
        % Converts the camera orientation to the direction cosine in the
        % reference coordinate system
        camera_ori = SV.A0 * rpy2dc(sensing_camera_param.mounting_angle);
        
        % Calvulate the inverse matrix
        camera_ori_inv = camera_ori';
        
        % Find the camera position in the reference coordinate system
        camera_pos = SV.R0 + SV.A0 * sensing_camera_param.mounting_position;
        
        % Transform the coordinates of the map points from the reference
        % coordinate system into RealSense camera coordinate system
        graspable_points_in_camera_coordinate = camera_ori_inv * (surface_param.graspable_points - camera_pos);
        
        
        for i = 1:length(surface_param.graspable_points)
            % Check the distance between the point and the camera in the x-axis
            % direction in the camera coordinate system.
            if graspable_points_in_camera_coordinate(1,i) > sensing_camera_param.fov_min_distance && graspable_points_in_camera_coordinate(1,i) < sensing_camera_param.fov_max_distance
                % Check if the angle made in xy coordinates is in the
                % horizontal fov of the camera.
                theta_xy = atan( graspable_points_in_camera_coordinate(2,i)/graspable_points_in_camera_coordinate(1,i) );
                if theta_xy < sensing_camera_param.fov_horizontal*0.5 && theta_xy > -sensing_camera_param.fov_horizontal*0.5
                    % Check if the angle made in xz coordinates is in the
                    % vertical fov of the camera.
                    theta_xz = atan(graspable_points_in_camera_coordinate(3,i)/graspable_points_in_camera_coordinate(1,i));
                    if theta_xz < sensing_camera_param.fov_vertical*0.5 && theta_xz > -sensing_camera_param.fov_vertical*0.5
                        % Add the point's positon to the matrix.
                        surface_param.sensed_graspable_points(:,i) = surface_param.graspable_points(:,i);
                    end
                end
            end
        end
    end
    if strcmp(sensing_camera_param.sensing_type, 'circle')
        % Convert the coordinates of graspable points to coordinates from the base center.
        graspable_points_pos_from_base_pos = surface_param.graspable_points - SV.R0;
        for i = 1:length(surface_param.graspable_points)
            % Calculate the distance from the base projection on the
            % ground to each graspable point
            distance_from_base_proj = sqrt(graspable_points_pos_from_base_pos(1,i)^2 + graspable_points_pos_from_base_pos(2,i)^2);
            if distance_from_base_proj < sensing_camera_param.circular_radius_from_base_pos
                surface_param.sensed_graspable_points(:,i) = surface_param.graspable_points(:,i);
            end
        end
    end
    
end
end