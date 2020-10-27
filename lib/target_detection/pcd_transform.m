%%%%%% Point Cloud Transformation
%%%%%% pcd_transform.m
%%%%%% 
%%%%%% Transform the Coordinates of a Point Cloud into a Coordinate System on the Least Squares Plane
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-16
%%%%%% Keigo Haji
%-------------------------------------------------------------------------
% pcd_transform.m transforms points from an orthonormal reference frame to a reference frame 
% oriented at the principal axis of the points. The x and y coordinates lie on the principal plane. 
% The z coordinates are normal to the principal plane.
%
% Function variables:
%
% OUTPUT
%   transformed_point_cloud_data: 3*n matrix
%   centroid_vector_of_plane: 1*3 vector, a positon vector indicating the centroid point of the least-squared plane
%   rotation_matrix: 3*3 matrix, a rotation matrix from pcd_around_robot to transformed_pcd
% 
% INPUT
%   input_point_cloud_data : 3*n matrix
% 
% [Note]
% For fractal terrain that is enenly distributed above and below the
% plane, there is no need to find the least-squares plane. Then you should
% set map_param.transform = "off" in the config file when the input pcd is
% the fractal terrain.
% Additionally, when executing target_detection.m in climb_main.m, this
% function should not be executed in target_detection.m, but should be
% executed in map_format_conversion.m to convert the terrain information
% beforehand. 
%-------------------------------------------------------------------------
function [transformed_pcd,centroid_vector_of_plane,rotation_matrix] = pcd_transform(input_pcd)
% principal axis theorem
% Calculates the least-squares plane of the pcd.
[normal_vector_of_plane, centroid_vector_of_plane] = pcd_least_squares_plane(input_pcd');

% Choose the correct direction of the z-axis. 
% if the normal vector is in a reverse direction, we will reverse convex shapes and concave shapes. 
% To avoid this, calculate the inner product of the normal vector of the plane and the centroid vector of the plane.
% If the normal vector is in the correct direction, the inner product will be < 0.
inner_product_of_centroid_and_normal_vector = dot(centroid_vector_of_plane,normal_vector_of_plane);
if inner_product_of_centroid_and_normal_vector > 0
    normal_vector_of_plane = - normal_vector_of_plane;
end

% Set the x and y vectors on the plane.
% Set the x and y in the same directions of camera's coordinate.
% Calculate the cross product of normal_vector_of_plane (z-axis) and the
% centroid vector.
y_vector_on_the_plane = cross(normal_vector_of_plane, centroid_vector_of_plane');
% normalize the size of the y vector.
y_vector_on_the_plane = y_vector_on_the_plane / norm(y_vector_on_the_plane);
% Calculate the cross product of normal_vector_of_plane (z-axis) and the y
% vector.
x_vector_on_the_plane = cross(y_vector_on_the_plane,normal_vector_of_plane);

% Make a rotation matrix.
rotation_matrix = [x_vector_on_the_plane y_vector_on_the_plane normal_vector_of_plane];

% Transform into the orthonormal basis on the plane.
% Note that we use inverse transformation here, not just the transformation equation.
transformed_pcd = rotation_matrix'* input_pcd + (-rotation_matrix')*centroid_vector_of_plane';



