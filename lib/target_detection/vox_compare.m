%%%%%% Compare voxel arrays
%%%%%% vox_compare.m
%%%%%% 
%%%%%% Compares elements of two voxel arrays
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-13
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% vox_compare.m compares two voxel arrays of the same size
% If the first voxel array has a solid voxel where the second voxel array has
% an empty voxel, we conclude that these arrays do not match.
%
% Function variables:
% 
% OUTPUT
%   matching_result: "regular","sub", or "no"
%                    "regular" means that subset_of_voxel_array certainly can be grasped. 
%                    "sub" means that subset_of_voxel_array possibly can be grasped. 
%                    "no" means that subset_of_voxel_array cannot be grasped. 
% INPUT
%   subset_of_voxel_array: l*m*n voxelgrid
%   gripper_mask: l*m*n voxelgrid
%   penalty_coefficient: Penalty factor for impaired solid voxels
%
%--------------------------------------------------------------------------
function matching_result = vox_compare(subset_of_voxel_array,gripper_mask,penalty_coefficient)
% Set the result.
matching_result = "regular";

% Counts the number of all solid voxels in the terrain array.
number_of_points = sum(subset_of_voxel_array(:));

% Count the number of solid voxels in the terrain array that lie inside
% the solid region of the gripper mask.
number_of_proper_points = sum(sum(sum(subset_of_voxel_array .* gripper_mask)));

% If the numbers are equal, it is judged once here as graspable.
if number_of_points == number_of_proper_points
    ;
else
    number_of_obstacle_points = number_of_points - number_of_proper_points;
    % If it is not zero, check if it is within the range of penalty_coefficient
    if number_of_proper_points < number_of_obstacle_points * penalty_coefficient
        matching_result = "no";
    else
        matching_result = "sub";
    end
end
      
    
end