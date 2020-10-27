%%%%%% Target_Detection_for_Climbing_Robots
%%%%%% target_detection.m
%%%%%% 
%%%%%% MAIN FUNCTION
%%%%%%
%%%%%% Extract Target Points from a Given Point Cloud
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-16
%%%%%% Keigo Haji
%
% Select points which can be grasped by the gripper from graspable points 
% 
% 
% Function variables:
%
% OUTPUT
%   graspable_points   : 3*N matrix contains position vectors of graspable points [m] (x;y;z)
% INPUT
%   graspable_points   : 3*M(>N) matrix contains position vectors of graspable points [m] (x;y;z)
%   gripper_or_peaks : "gripper" or "peaks"
%                       If input string is "gripper", then detect the
%                       targets based on the gripper configuration.
%                       If input string is "peaks", then detect the peaks
%                       of the convex shapes as the targets.
%                       This string is defined in config_environment_param.m 
%                       as environment_param.graspable_points_detection_type
%   gripper_param : parameters for the gripper-mask
%   map_param     : parameters and settings for creating the terrain
%                   matrix
%   matching_settings : settings for matching


function graspable_points = target_detection(graspable_points, gripper_or_peaks, gripper_param, map_param, matching_settings)
% Extract only x,y and z positions of points
graspable_points = graspable_points(1:3,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set and Load parameters of the gripper configuration, map parameters, and
% matching settings. 
% This line is now executed in config_simulation.m, and we can change in
% config_USER_param.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [gripper_param, map_param, matching_settings] = config_target_detection_param();


% Delete the points which located farther than outliers_threshold if the
% setting is "ON"
if map_param.delete_outliers == "on"
    [graspable_points] = pcd_delete_outliers(graspable_points,map_param.outliers_threshold);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The following two lines, pcd_transform and pcd_interpolate, are now
% executed before the simulation execution by map_format_conversion.m.
% The reason for this is that we use the grid's xyz form instead of the 3*n
% matrix in main simulation.  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transform the points from an orthonormal reference frame to a reference
% frame on the least-squared plane. 
if map_param.transform == "on"
    [graspable_points,centroid_vector_of_plane,rotation_matrix] = pcd_transform(graspable_points);
end
% Interpolate the points to avoid occlusions, if the interpolation
% setting is ON. 
if map_param.interpolation == "on"
    [graspable_points,~,~,~, grid_size] = pcd_interpolate(graspable_points,map_param.interpolation_method);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Offset the points 
[graspable_points,offset_vector] = pcd_offset(graspable_points);

% Make a terrain matrix from the points 
[terrain_matrix] = pcd_voxelize(graspable_points, map_param.voxel_size, map_param.voxel_threshold);

% Make a gripper-mask
[gripper_mask] = grippermask(gripper_param, map_param.voxel_size, gripper_or_peaks);

% Check the "grippability" and select the graspable points
[voxel_coordinates_of_graspable_points] = voxel_matching(terrain_matrix,gripper_mask,matching_settings, gripper_or_peaks);

% Select the representative points from the matching graspable points if
% the setting is "on"
if matching_settings.colony == "on"
    switch map_param.interpolation
        case "on"
            [voxel_coordinates_of_graspable_points] = colony_search(voxel_coordinates_of_graspable_points, matching_settings, map_param.voxel_size ,grid_size);
        case "off"
            [voxel_coordinates_of_graspable_points] = colony_search(voxel_coordinates_of_graspable_points, matching_settings, map_param.voxel_size);     
    end
end

% Re-transform the frame
if map_param.transform == "on"
   [graspable_points] = pcd_re_transform(voxel_coordinates_of_graspable_points,map_param.voxel_size,offset_vector,rotation_matrix,centroid_vector_of_plane);
elseif map_param.transform == "off"
   [graspable_points] = pcd_re_transform(voxel_coordinates_of_graspable_points,map_param.voxel_size,offset_vector);
end
   
end


