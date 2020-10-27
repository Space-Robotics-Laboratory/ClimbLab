%%%%%% MAP Format Conversion
%%%%%% pcd_transfrom.m & pcd_interpolate.m 
%%%%%% 
%%%%%% Convert PCD from Realsense into xyz grid format
%%%%%% 
%%%%%% Created 2020-10-16
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-16
%%%%%% Keigo Haji
%-------------------------------------------------------------------------
% map_format_conversion.m changes the point cloud data of the terrain
% described as a 3Å~N matrix to a map in gridded format. 
% In the process, the point cloud is rotated in the least-squares plane and
% the selected interpolation method is used to interpolate the point cloud
% information.
% 
% Function variables:
% 
% OUTPUT
%   x y z : A map in grid format with the input points rotated and interpolated.
%           x and y are vectors, and z is the matrix.
% 
% INPUT
%   pcd : Raw Point Cloud Data from Realsense, 3*n matrix
%   interpolation_method : "linear" or "natural" 
% 
% [Note]
% This function is not executed in climb_main.m.
% This function is used to convert a new PCD file acquired by a RealSense
% camera from a 3*N matrix to the grid format used by ClimbLab. 
% After execution, check the results to see if the terrain data is not
% upside down.  
% If there is no problem, pack the three variables of xyz together and save
% them as a .mat file. 
%-------------------------------------------------------------------------
function [x,y,z] = map_format_conversion(pcd,interpolation_method)

% If you use the map which is already transformed into the
% least-square plane, you should comment out the following line.
[pcd,~,~] = pcd_transform(pcd);

[~,x,y,z,~] = pcd_interpolate(pcd,interpolation_method);
 
end
