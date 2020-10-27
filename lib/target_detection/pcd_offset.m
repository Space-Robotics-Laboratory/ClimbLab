%%%%%% Parallel Translation
%%%%%% pcd_offset.m
%%%%%% 
%%%%%% Translate the point clouds
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-09-23
%%%%%% Keigo Haji
%-------------------------------------------------------------------------
% pcd_offset.m offsets the points of A so that the smallest x,y and z coordinate
% is equal to zero.
% If offset_vector is inputted, pclzero.m offsets the inputted pcd of A by the inputted offset_vector.
%
% Function variables:
%
% OUTPUT
%   B: 3*n matrix, output coordinates after translation
%   offset_vector: 1*3 vector, contains offset values in xyz directions
% 
% INPUT
%   A: 3*n matrix, input coordinates
%-------------------------------------------------------------------------
function [offset_pcd,offset_vector] = pcd_offset(transformed_pcd,offset_vector)
if nargin == 1
    
    %Calculate offset values in x-y-z directions
    offset_vector = min(transformed_pcd,[],2);
    
    %Offset
    offset_pcd= [transformed_pcd(1,:)-offset_vector(1); transformed_pcd(2,:)-offset_vector(2); transformed_pcd(3,:)-offset_vector(3)];

elseif nargin == 2 
    offset_pcd = transformed_pcd - offset_vector;
     
    
end
