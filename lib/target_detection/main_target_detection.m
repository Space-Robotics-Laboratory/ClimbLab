%%%%%% MAIN FILE ofTarget_Detection_for_Climbing_Robots
%%%%%% main_target_detection.m
%%%%%% 
%%%%%% MAIN FILE
%%%%%%
%%%%%% Extract Target Points from a Given Point Cloud
%%%%%% 
%%%%%% Created 2021-04-10
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-22
%%%%%% Keigo Haji
%%%%%% 
%%%%%% This files are used for target detection outside of the cilmblab.
%%%%%% Before executing in climblab, we can run the target detection as
%%%%%% same situation in cilmblab and check the result.
%
% [Note]
% This MAIN file is not executed in main_sim.m.
% This is used to convert a new PCD file acquired by a RealSense
% camera from a 3*N matrix to the grid format used by ClimbLab. 
% After execution, check the results to see if the terrain data is not
% upside down, and the algorithm detect proper targets on the terrain.
% If there is no problem, pack the three variables of xyz together and save
% them as a .mat file. 
% [Note 2]
% When you added new bouldering holds map taken by RealSense camera in
% ClimbLab, you must include 'climbing_holds' in the .mat file name. This
% is because we switch how to plot the surface in vis_surface depending on
% the name of the map. 
%
clc; clear; close all; 
tic; 
global x y z;

%% Load config file of config_target_detection_testing_param
[gripper_param, map_param, matching_settings, gripper_or_peaks, environment_param, ani_settings] = config_target_detection_testing_param();

%% Load designated PCD file and change the format from PCD to xyz

% Load a point cloud from a PCD format file
rawPointCloudData = loadpcd(map_param.pcd_file_name);
rawPointCloudData = rawPointCloudData(1:3,:);


% Change the format from the 3*N matrx to the xy vectors and z matrix format. 
    % [x,y,z] = map_format_conversion(rawPointCloudData,map_param.interpolation_method);
[pcd,~,~] = pcd_transform(rawPointCloudData);
%%% Delete lower points probably located on the wall
if map_param.delete_lower_points_in_transformed_PCD == "on"
    pcd = target_delete(pcd, map_param.delete_lower_points_threshold);
end
[~,x,y,z,~] = pcd_interpolate(pcd,map_param.interpolation_method);


%% Reshape the grid data to the 3*N matrix like ini_graspable_points

% Reshape the data to the 3*N matrix like ini_graspable points.
[n,m]=size(z);
surface_param.graspable_points=zeros(3,n*m);
[X,Y] = meshgrid(x,y);
surface_param.graspable_points(1,:)=reshape(X,1,n*m);
surface_param.graspable_points(2,:)=reshape(Y,1,n*m);
surface_param.graspable_points(3,:)=reshape(z,1,n*m);

% Set the min and max to adjust color of vis_surface like ini_surface
surface_param.min = min(surface_param.graspable_points(3,:));
surface_param.max = max(surface_param.graspable_points(3,:));

% Extract only x,y and z positions of points
graspable_points = surface_param.graspable_points(1:3,:);


%% Execute target detection
%% The following lines are completely same as target_detection.m which is executed in ini_graspable_points


% Delete the points which located farther than outliers_threshold if the
% setting is "ON"
if map_param.delete_outliers == "on"
    [graspable_points] = pcd_delete_outliers(graspable_points,map_param.outliers_threshold);
end

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


% Delete the targets whose z positions are less than threshold
if matching_settings.delete_lower_targets == "on"
    [graspable_points] = target_delete(graspable_points,matching_settings.delete_lower_targets_threshold);
else
    ;
end

%% Inset the graspable_points to the surface_param like ini_graspable_points
% Insert the graspable_points to display them in vis_graspable_points
surface_param.graspable_points = graspable_points;


%% Visualize the result like main_sim visualization

inc = environment_param.inc;
figure(1); clf; hold on; axis equal;
vis_surface(inc, ani_settings, environment_param, surface_param);
vis_graspable_points(surface_param,inc,ani_settings);


toc
% EOF