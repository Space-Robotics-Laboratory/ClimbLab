%%%%%% Extract Voxel Array
%%%%%% vox_extract.m
%%%%%% 
%%%%%% Extract a small voxel array from the big voxel array
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-09-23
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% vox_extract.m extracts an array of voxels of the specified size from a larger voxel array.
% We want to compare the gripper mask with the extracted voxel array ofÅ@the same size as the gripper mask.
%
% Function variables:
% 
% OUTPUT
%   extractev_voxel_array: i*i*j matrix, extracted 3-dimensional voxel array (i*i*j matrix)
% 
% INPUT
%   voxel_array: m*n*l matrix, 3-dimensional voxelgrid 
%   position_of_reference_point: 1*3 vector, position of the extracted voxel array, xyz 
%   size_of_extracting : 1*3 vector, size of the of the extracted voxelgrid ,i*i*j
%
%--------------------------------------------------------------------------
function extracted_voxel_array = vox_extract(voxel_array,position_of_reference_point,size_of_extracting)

extracted_voxel_array = voxel_array(position_of_reference_point(1) : position_of_reference_point(1)+size_of_extracting(1)-1,...
                                    position_of_reference_point(2) : position_of_reference_point(2)+size_of_extracting(2)-1,...
                                    position_of_reference_point(3) : position_of_reference_point(3)+size_of_extracting(3)-1);
   
end
