%%%%%% Least Squares Plane
%%%%%% pcd_least_squares_plane.m
%%%%%% 
%%%%%% Calculate the least squares plane of the point clouds
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-09-23
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% pcd_least_squares_plane calculates the least-squares plane of the input point set.
%
% Function variables:
% 
% OUTPUT:
%   normal_vector_of_plane : a unit (column) vector normal to the least-squares plane
%   centroid_vector_of_plane : a point on the least-squares plane
% 
% INPUT:
%   point_cloud_data : n * 3 matrix where each row is a sample point, and each column
%                      corresponds to x,y,z positons
%
%-------------------------------------------------------------------------
function [normal_vector_of_plane, centroid_vector_of_plane] = pcd_least_squares_plane(pcd_around_robot)

% Calculate the mean of pcd as a point on the least-squares plane.
centroid_vector_of_plane = mean(pcd_around_robot);

% Calculate deviations between points and the centroid point.
% bsxfun: Apply element-wise operation to two arrays with implicit expansion enabled
deviation_pcd = bsxfun(@minus ,pcd_around_robot,centroid_vector_of_plane);


% Compute eigenvectors and eigenvalues of deviation matrix
[right_eigenvectors_Matrix, diagonal_matrix_of_eigenvalue] = eig(deviation_pcd'*deviation_pcd);

% Extract the output from the eigenvectors
% The eigenvector corresponding to the minimum eigenvalue corresponds to the normal vector of the plane.
[~,index_of_sorted_eigenvalues] = sort(diag(diagonal_matrix_of_eigenvalue));
sorted_right_eigenvector_matrix = right_eigenvectors_Matrix(:,index_of_sorted_eigenvalues);
normal_vector_of_plane = sorted_right_eigenvector_matrix(:,1);

end