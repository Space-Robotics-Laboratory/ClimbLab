%%%%%% Clip Voxel Array
%%%%%% vox_clip.m
%%%%%% 
%%%%%% Clip the voxel array and set 0
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-09-23
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% vox_clip.m clips the searched array at all sides by setting the values to zero.
% We want to prevent the gripper mask from protruding from the voxel array during the matching algorithm. 
% We can limit the range of searching by using this function because the values in the area around the outer edges are set 0. 
% Then we will compare  3-dimensional arrays based on solid(1) voxels located in the inside area having room for gripper-mask.
%
% Function variables:
%
% OUTPUT
%   searchVoxelArray: search voxelgrid having the room of a gripper mask
%                     around the outer edges
% INPUT
%   searchVoxelArray: search voxelgrid
%   x: number of surfaces set 0 in x-direction
%   y: number of surfaces set 0 in y-direction
% 
%--------------------------------------------------------------------------
function search_voxel_array = vox_clip(search_voxel_array,x,y)

% Crop in x-direction 
search_voxel_array(end-x+1:end,:,:) = 0;
search_voxel_array(1:x,:,:)         = 0;

% Crop in y-direction
search_voxel_array(:,end-y+1:end,:) = 0;
search_voxel_array(:,1:y,:)         = 0;

end