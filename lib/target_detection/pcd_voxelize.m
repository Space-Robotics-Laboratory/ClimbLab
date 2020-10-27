%%%%%% Voxelize
%%%%%% pcd_voxelize.m
%%%%%% 
%%%%%% Makes the point cloud data voxelized.
%%%%%% 
%%%%%% Created 2020-09-02
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-16
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% pcd_voxelize.m makes the point cloud data voxelized.
% Change point-cloud-data of the orthonormal coordinate system into a
% 3-dimensional array composed 0 and 1.
% 
% Function variables:
% 
% OUTPUT
%   terrain_matrix: n*m*l 3-dimensional array
%                               voxel array made from point cloud data
%                               If a point exists in a certain voxel, the voxel is 1
%                               If the voxel is empty, the voxel is 0
%
%
% INPUT
%   offset_point_cloud_data: 3*n matrix
%                            Each column corresponds to a point.     
%                            1st, 2nd, and 3rd rows indicate x, y, and z
%                            positions.
%                            All of the positions are bigger than 0.
%   voxel_size: length of one side of voxel [m]   
%   voxel_threshold:   distance from center point of voxel [m]
%                      If a centroid of points existing in the voxel is
%                      farther than this value, the voxel is set 0 even
%                      though there are some points in the voxel
%
% [Note]
% Note that the output voxel array made from point-cloud is upside down.
% The voxels whose third subscripts are lower exist higher positions in the point
% cloud data.
% However this is not a problem, because the gripper mask is also upside down,
% so they are in same direction.
% Thus we correct the position after voxel-matching-algorithm at the end of
% voxel_matching.m to output figures and coordinates correctly.
%--------------------------------------------------------------------------
function [terrain_matrix] = pcd_voxelize(offset_pcd, voxel_size,voxel_threshold)
%=== preperations =========================================================

if nargin == 1
    voxel_size = 1;
    voxel_threshold = 1;
end

% Tranform coordinates so that the max-z-coordinate becomes zero
z_max = max(offset_pcd(3,:));
offset_pcd = [offset_pcd(1,:); offset_pcd(2,:); z_max - offset_pcd(3,:)];


%=== processing ===========================================================

% Project the points in each voxel on the voxel's surfaces close to the origin
% distanceFromTheCornerOfEachVoxelToPointsInAxialDirection : 3*n matrix
distance_from_the_corner_of_each_voxel_to_points = mod(offset_pcd,voxel_size);
corner_positions_of_voxels_containing_points = offset_pcd - distance_from_the_corner_of_each_voxel_to_points;

% Calculate the deviation to the voxel center of each point
% deviationFromTheCenterOfEachVoxelToPointsInAxialDirection : 3*n matrix
deviation_from_the_centers_of_each_voxel_to_points = voxel_size/2 - distance_from_the_corner_of_each_voxel_to_points;

% Move to the points from the coner to the center of the corresponding voxel
center_positions_of_voxels_containing_points = corner_positions_of_voxels_containing_points + voxel_size/2;


% Remove duplicates voxels, because we just want know which voxel has points
% [C,ia,ic] = unique(A,'rows','stable') :Find the unique rows of A based on the data
%                                       'stable' sets the values in C to have the same order as in A. 
%                                       C = A(ia) and A = C(ic)
% centerPositionsOfVoxelsContainingPoints' = centerPositionsOfUniqueVoxelsContainingPoints(indexNumberOfVoxelContainigPoints)
% centerPositionsOfUniqueVoxelsContainingPoints : 3*m matrix
[center_positions_of_unique_Voxels_containing_points, ~, index_list_of_voxels_containig_points] = unique(center_positions_of_voxels_containing_points','rows','stable');
center_positions_of_unique_Voxels_containing_points = center_positions_of_unique_Voxels_containing_points';


%Count duplicates in each voxel to calculate centroid point
number_of_points_in_each_voxel = histc(index_list_of_voxels_containig_points, unique(index_list_of_voxels_containig_points));

% Accumulate distances to the center in each voxel
% deviation_accumulation_in_each_voxel : 3*m matrix
%   each colum corresponds to an unique voxel containg points
%   each row corresponds to Accumulation of distances from the center to the points in the voxel.
%   1st, 2nd, and 3rd corresponds to x, y, and z axial directions.
% A = accumarray(subs,val) returns array A by accumulating elements of vector val using the subscripts subs. 
%   If subs is a column vector, then each element defines a corresponding subscript in the output, 
%   which is also a column vector. The accumarray function collects all elements of val 
%   that have identical subscripts in subs and stores their sum in the location of A corresponding to that subscript
% (for index i, A(i)=sum(val(subs(:)==i))).
%   Elements of A whose subscripts do not appear in subs are equal to 0.
deviation_accumulation_in_each_voxel = ...
    [accumarray(index_list_of_voxels_containig_points, deviation_from_the_centers_of_each_voxel_to_points(1,:)')...
    accumarray(index_list_of_voxels_containig_points, deviation_from_the_centers_of_each_voxel_to_points(2,:)')...
    accumarray(index_list_of_voxels_containig_points, deviation_from_the_centers_of_each_voxel_to_points(3,:)')]';


% Calculate mean disviation to the center in each voxel
mean_deviation_of_each_voxel_in_axial_direction = deviation_accumulation_in_each_voxel ./ number_of_points_in_each_voxel';

% Remove voxels with too big mean distance in each voxel
mean_distance_of_each_voxel = abs(mean_deviation_of_each_voxel_in_axial_direction);
center_positions_of_solid_oxels = center_positions_of_unique_Voxels_containing_points...
                                    (:,mean_distance_of_each_voxel(1,:) < voxel_threshold...
                                    & mean_distance_of_each_voxel(2,:) < voxel_threshold...
                                    & mean_distance_of_each_voxel(3,:) < voxel_threshold);

% Preallocate voxelgrid
% Change values into integers to use the values as subscripts in 3-dimensional array  
subscripts_of_solid_voxels = int64((center_positions_of_solid_oxels - voxel_size/2) / voxel_size+1);

% Make voxel space as 3-dimensional array composed of 0 
dimensional_size_of_voxel_space = [max(subscripts_of_solid_voxels(1,:)),...
                             max(subscripts_of_solid_voxels(2,:)),...
                             max(subscripts_of_solid_voxels(3,:))];
terrain_matrix = zeros(dimensional_size_of_voxel_space);

% Transform solid voxels' indices from subindices
index_list_of_solid_voxels = sub2ind(dimensional_size_of_voxel_space, subscripts_of_solid_voxels(1,:),subscripts_of_solid_voxels(2,:),subscripts_of_solid_voxels(3,:));

% Set solid voxels as 1
terrain_matrix(index_list_of_solid_voxels) = 1;


% If you want to see the point cloud data after voxelization, refer to the
% following command. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% size_of_terrain_matrix = size(terrain_matrix);
% index_list_of_solid_voxels_in_terrain_matrix = find(terrain_matrix);
% number_of_solid_voxels = length(index_list_of_solid_voxels_in_terrain_matrix);
% solid_voxels_matrix = zeros(3,number_of_solid_voxels);
% [x_subscripts_of_all_solid_voxels,y_subscripts_of_all_soid_voxels,z_subscripts_of_all_solid_voxels] = ind2sub([size_of_terrain_matrix(1),size_of_terrain_matrix(2)],index_list_of_solid_voxels_in_terrain_matrix);
% solid_voxels_matrix(1,:) = x_subscripts_of_all_solid_voxels';
% solid_voxels_matrix(2,:) = y_subscripts_of_all_soid_voxels';
% solid_voxels_matrix(3,:) = z_subscripts_of_all_solid_voxels';
% max_z_subscripts = max(z_subscripts_of_all_solid_voxels);
% solid_voxels_matrix(3,:) = -solid_voxels_matrix(3,:) + max_z_subscripts;
% scatter3(solid_voxels_matrix(1,:), solid_voxels_matrix(2,:),solid_voxels_matrix(3,:),'s','k')
% axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end