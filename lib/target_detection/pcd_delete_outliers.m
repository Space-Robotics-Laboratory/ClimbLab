%%%%%% Delete Outliers
%%%%%% pcd_delete_outliers.m
%%%%%% 
%%%%%% Delete points of outliers 
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-13
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% pcd_delete_outliers.m deletes points from inputted point clouds which locate farther than OUTLIERS_THRESHOLD 
% to think only near points from our robot.
% This is because computers cannot make huge voxel grids because of memory size and also least-square
% planes make sense for small areas.
%
% Function variables:
% 
% OUTPUT
%   graspable_points : 3*m matrix, some farther points are deleted from
%                      inputed points data
%   outliers_threshold : [m]  Delete points located farther than this value  
% INPUT
%   graspable_points : 3*n matrix, 1st, 2nd and 3rd rows indicate x,y, 
%                      and z positons. Each column corresponds to a point
%
%--------------------------------------------------------------------------
function [graspable_points] = pcd_delete_outliers(graspable_points,outliers_threshold)

% Prepare for FOR-LOOP
[~,total_number_of_points] = size(graspable_points);
index_number_of_points = zeros(1,total_number_of_points);


% list points locate farther than OUTLIERS_THRESHOLD either in x,y or z direction
for i = 1:total_number_of_points
    if (abs(graspable_points(1,i)) > outliers_threshold) || (abs(graspable_points(2,i)) > outliers_threshold) || (abs(graspable_points(3,i)) > outliers_threshold) 
        index_number_of_points(i) = i;
    end
end

% delete 0 columns in index vector
index_number_of_points = index_number_of_points(:,any(index_number_of_points,1));

% delete corresponding colums in point cloud data around robot
graspable_points(:,index_number_of_points) = [];
end