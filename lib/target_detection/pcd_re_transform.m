%%%%%% Re-Transform
%%%%%% pcd_re_transform.m
%%%%%% 
%%%%%% Return to the original coordinate system
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-09-26
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% pcd_re_transform.m returns the coordinates of the voxel array to the original input coordinate system.
% 
% Function variables:
% 
% OUTPUT
%   graspable_points : 3*n matrix,
%                      Each column represents a point
%                      Each row an xyz coordinate in the original coordinate system
% 
% INPUT
%   voxel_array_of_graspable_points: 4*n matrix
%                                    Each column indicates a matched voxel. 
%                                    1st, 2nd, and 3rd raws indicate x y z subscripts in voxel array.
%   rotation_matrix : 3*3 matrix which were used in pcd_transform
%   centroid_vector_of_plane : 1*3 vector which indicates the point located on
%                              least-squared plane
%   voxel_size: length of one side of voxel used in voxelize [m]
%   offset_vector: 1*3 vector used in pcd_offset
%
%--------------------------------------------------------------------------
function [graspable_points] = pcd_re_transform(voxel_coordinates_of_graspable_points,voxel_size,offset_vector,rotation_matrix,centroid_vector_of_plane)

if isempty(voxel_coordinates_of_graspable_points) == 1
    graspable_points = zeros(3,0);
else
    
    % Return the dimension from voxel dimension to the real world [m]
    graspable_points(1:3,:) = voxel_coordinates_of_graspable_points(1:3,:)*voxel_size;
    
    % Move offset values in reverse directions
    graspable_points(1:3,:) = [graspable_points(1,:)+offset_vector(1); graspable_points(2,:)+offset_vector(2); graspable_points(3,:)+offset_vector(3)];
    
    % if map_param.transform = "on"
    if nargin == 5
        % Transform the coordinate on the least-squared plane to the natural coordinate
        graspable_points(1:3,:) = rotation_matrix*graspable_points(1:3,:) + centroid_vector_of_plane';
    end
    
end

end