%%%%%% MAP Interpolation
%%%%%% pcd_interpolate.m
%%%%%% 
%%%%%% Interpolate the point clouds
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-16
%%%%%% Keigo Haji
%-------------------------------------------------------------------------
% pcd_interpolate.m interpolates the inputted points by Delaunay
% trangulation method supported by the MATLAB function of griddata.
% 
% Function variables:
% 
% OUTPUT
%   interpolated_pcd : 3*m matrix
%   x_grid_vector, y_grid_vector : 1*j vector, 1*k vectoir
%   z_pcd_in_grid_format : k*j matrix
%   grid_size
% 
% INPUT
%   inputted_pcd : 3*n matrix
%   interpolation_method : "linear" or "natural" set in the config file 
% 
% [Note]
% When executing target_detection.m in climb_main.m, this function should
% not be executed in target_detection.m, but should be executed in
% map_format_conversion.m to convert the terrain information beforehand. 
% If you want to use surf function, you can use as follows,
% surf(x_pcd_in_grid_format,y_pcd_in_grid_format, z_pcd_in_grid_format);
%-------------------------------------------------------------------------
function [interpolated_pcd,x_grid_vector,y_grid_vector,z_pcd_in_grid_format,grid_size] = pcd_interpolate(input_pcd,interpolation_method)
        
% Count the number of input points
[~,number_of_points] = size(input_pcd);      
                
% Change the format of the numbers to work the following MATLAB functions
x = double(input_pcd(1,:));
y = double(input_pcd(2,:));
z = double(input_pcd(3,:));

% Check the size of the map
x_width = max(x) - min(x);
y_width = max(y) - min(y);

% Set the width of the grid as grid_size so that the number of points
% after interpolation is approximately the same as the input points.
% Assuming a point cloud file taken from an oblique angle by a RealSense 
% camera, the topographic information looks like a trapezoid depending on 
% the angle of view of the camera, and further occlusion occurs.
% We set 5/3 as a coefficient appropriately so that when the points were 
% interpolated in that situation, they would have roughly the same number
% of points when the points were interpolated.
density_of_points = number_of_points/(x_width * y_width);
grid_size = 1/round(sqrt(density_of_points*(5/3)),4);

x_grid_vector = min(x):grid_size:max(x);
y_grid_vector = min(y):grid_size:max(y);
% Divide the area into a mesh
[x_pcd_in_grid_format,y_pcd_in_grid_format] = meshgrid(x_grid_vector, y_grid_vector);

% Interpolate points by Delaunay trangulation on each mesh grid point
z_pcd_in_grid_format = griddata(x, y, z, x_pcd_in_grid_format, y_pcd_in_grid_format, interpolation_method);


% Change the points of grid format into a matrix as 3*m.
[m,n] = size(z_pcd_in_grid_format);
interpolated_pcd = zeros(3,m*n);
k = 1;
for i = 1:m
    for j = 1:n
        if isnan(z_pcd_in_grid_format(i,j)) == 0
        interpolated_pcd(1,k) =  x_pcd_in_grid_format(i,j);
        interpolated_pcd(2,k) =  y_pcd_in_grid_format(i,j);
        interpolated_pcd(3,k) =  z_pcd_in_grid_format(i,j);
        k = k + 1;
        end
    end
end

% Delete the 0 columns
interpolated_pcd = interpolated_pcd(:,any(interpolated_pcd,1));
end

