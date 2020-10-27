%%%%%% Voxel Matching Algorithm for Climbing
%%%%%% voxel_matching.m
%%%%%% 
%%%%%% Sequentially match the gripper-mask to the terrain matrix
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-13
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% voxel_matching.m finds subset voxel arrays in a major voxel array that matches a gripper mask voxel array.
%
% Function variables:
%
% OUTPUT
%   voxel_array_of_graspable_points : 4*n matrix, containing subscripts of "grippable" points and "grippability" at those points  
%                                     1st,2nd, and 3rd rows indicate subscripts in x-y-z directionss
%                                     4th row indicates the number of solid
%                                     voxels, but that of the sub-graspable
%                                     points is 1
% INPUT
%   terrain_matrix: 3-dimensional array composed of 0 and 1
%   gripper_mask: 3-dimensional array of gripper mask composed of 0 and 1
%   matching_settings: structure parameters of detection settings set in the config file
%--------------------------------------------------------------------------

function [voxel_coordinates_of_graspable_points] = voxel_matching(terrain_matrix,gripper_mask,matching_settings,gripper_or_peaks)
%=== preparations =========================================================

% Copy inputted terrain data
searching_voxel_array = terrain_matrix;
size_of_voxel_array = size(terrain_matrix);

% Measure the size of the gripper_mask
size_of_gripper_mask = size(gripper_mask);
half_size_of_gripper_mask = round(size_of_gripper_mask/2);


% Save z subscript of solid voxels
index_list_of_solid_voxels_in_full_voxel_array = find(terrain_matrix);
[~,~, z_subscripts_of_all_solid_voxels] = ind2sub([size_of_voxel_array(1),size_of_voxel_array(2)],index_list_of_solid_voxels_in_full_voxel_array);


% Insert empty voxel layers in z-direction only bottom
extended_empty_voxel_layers = zeros(size_of_voxel_array(1),size_of_voxel_array(2),size_of_gripper_mask(3));
terrain_matrix = cat(3,terrain_matrix, extended_empty_voxel_layers);


% Crop edges of the searching voxel array
% Prevent the gripper mask from protruding from the voxel array during the matching algorithm. 
% Use another function of voxel_crop.m
searching_voxel_array = vox_clip(searching_voxel_array,...
                                half_size_of_gripper_mask(1)+1,...
                                half_size_of_gripper_mask(2)+1);

%Find all ones, solid voxels, in the search voxel array
index_list_of_solid_voxels_in_searching_voxel_array = find(searching_voxel_array);
% Change indexes to subscripts of solid voxels
[x_subscripts_of_searching_solid_voxels,y_subscripts_of_searching_solid_voxels,z_subscripts_of_searching_solid_voxels]...
    = ind2sub([size_of_voxel_array(1),size_of_voxel_array(2)], index_list_of_solid_voxels_in_searching_voxel_array);


% Correct the positions of searching voxels to use extvox.m and compvox.m
% reasonablly
x_subscripts_of_searching_solid_voxels = x_subscripts_of_searching_solid_voxels - half_size_of_gripper_mask(1);
y_subscripts_of_searching_solid_voxels = y_subscripts_of_searching_solid_voxels - half_size_of_gripper_mask(2);

% Prepare for loop
number_of_solid_voxels_in_searching_voxel_array = length(index_list_of_solid_voxels_in_searching_voxel_array);
regular_matching_list = zeros(4,number_of_solid_voxels_in_searching_voxel_array);
searching_solid_voxels_map = zeros(4,number_of_solid_voxels_in_searching_voxel_array);
sub_matching_list = zeros(4,number_of_solid_voxels_in_searching_voxel_array);

%=== VMAC =================================================================

if gripper_or_peaks == "gripper"
    for index_of_voxel_being_compared = 1:number_of_solid_voxels_in_searching_voxel_array
        
        % Extract subset in the same size of the gripper mask from the data voxel array
        % Use another function of vox_extract.m
        subset_of_voxel_array ...
            = vox_extract(terrain_matrix,...
            [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared),y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared),z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared)],...
            size_of_gripper_mask);
        searching_solid_voxels_map(:,index_of_voxel_being_compared)...
            = [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(1);...
            y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(2);...
            z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared);...
            0];
        
        % Compare the two arrays and check whether they match or not
        % Use another function of voxel_compare.m
        matching_result =  vox_compare(subset_of_voxel_array,gripper_mask,matching_settings.penalty_coefficient);
        if matching_result == "regular"
            % If output of compvox is "regular", it means that the subset
            % of point cloud data goes into the gripper mask and it can be
            % gripped by the gripper.
            % Count number of points in the grippermask.
            number_of_matching_voxels = sum(subset_of_voxel_array(:));
            
            % Check whether the number of points is bigger than the matching
            % threshold
            if number_of_matching_voxels > matching_settings.threshold
                % Save number of solid voxels and grippable point as the
                % position of natural voxel array of point cloud data
                regular_matching_list(:,index_of_voxel_being_compared) = [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(1);...
                    y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(2);...
                    z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared);...
                    number_of_matching_voxels];
            end
        elseif matching_result == "sub"
            number_of_matching_voxels = sum(subset_of_voxel_array(:));
            if number_of_matching_voxels > matching_settings.threshold
                % Set the 4th row of sub-grippable points as 1 to distinguish
                % regular-grippable targets.
                sub_matching_list(:,index_of_voxel_being_compared) = [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(1);...
                    y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(2);...
                    z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared);
                    1];
                %               number_of_matching_voxels];
            end
        end
    end
    
elseif gripper_or_peaks == "peaks"
    % If only peaks are to be detected, the positions of the solid voxels to be
    % referenced is shifted upwards.
    extended_empty_voxel_layers = zeros(size_of_voxel_array(1),size_of_voxel_array(2),size_of_gripper_mask(3)+1);
    terrain_matrix = cat(3, extended_empty_voxel_layers,terrain_matrix);
    
    for index_of_voxel_being_compared = 1:number_of_solid_voxels_in_searching_voxel_array
        
        % Extract subset in the same size of the gripper mask from the data voxel array
        % Use another function of vox_extract.m
        % By shifting the reference voxels upwards at the above lines, we
        % will extract the upper voxel array of the solid voxel. 
        subset_of_voxel_array ...
            = vox_extract(terrain_matrix,...
            [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared),y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared),z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared)],...
            size_of_gripper_mask);
        searching_solid_voxels_map(:,index_of_voxel_being_compared)...
            = [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(1);...
            y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(2);...
            z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared);...
            0];
        
        % Compare the two arrays and check whether they match or not
        % Use another function of voxel_compare.m
        matching_result =  vox_compare(subset_of_voxel_array,gripper_mask,matching_settings.penalty_coefficient);
        if matching_result == "regular"
            number_of_matching_voxels = sum(subset_of_voxel_array(:));
            
            % The gripper_mask for the peaks is a three-dimensionla array
            % of all elements of 1. Comparing the upper part of a solid
            % voxel and this gripper_mask, when the number of matching
            % voxels is 0, that is, the solid voxel is a peak of the
            % partial terrain.
            if number_of_matching_voxels == 0
                % Save number of solid voxels and grippable point as the
                % position of natural voxel array of point cloud data
                regular_matching_list(:,index_of_voxel_being_compared) = [x_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(1);...
                    y_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared) + half_size_of_gripper_mask(2);...
                    z_subscripts_of_searching_solid_voxels(index_of_voxel_being_compared);...
                    number_of_matching_voxels];
            end
        end
        % When the setting is 'peaks', currently we do not consider the
        % sub-targets.
    end
end


% Crop the remaining 0 column
regular_matching_list = regular_matching_list(:,any(regular_matching_list,1));
sub_matching_list = sub_matching_list(:,any(sub_matching_list,1));


% Correct the position of the voxel array of the terrain matrix
% because we make the voxel array in the reverse direction in pcd_voxelize.m
max_z_position = max(z_subscripts_of_all_solid_voxels);
regular_matching_list(3,:) = -regular_matching_list(3,:) + max_z_position;
sub_matching_list(3,:) = -sub_matching_list(3,:) + max_z_position;


if matching_settings.submatching == "off"
    voxel_coordinates_of_graspable_points = regular_matching_list;
elseif  matching_settings.submatching == "on"
    voxel_coordinates_of_graspable_points = [regular_matching_list sub_matching_list];
end



end