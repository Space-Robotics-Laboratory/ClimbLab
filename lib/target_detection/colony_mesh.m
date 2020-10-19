%%%%%% Colony-Mesh
%%%%%% colony_mesh.m
%%%%%% 
%%%%%% Divide the colony into meshes and select the candidate points for the grasp.
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-12
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% colony_mesh.m is a function which sections colony into meshes.
% We choose a representative voxel from the colony and calculate centroid 
% points in meshes as sub-representative points.
% We do not think the centroid in the mesh which has the representative voxel  
%
% Function variables:
%
% OUTPUT
%   target_list_of_colony: 4*m matrix
%                          representative and sub-representative points in the input colony
%   number_of_targets_in_colony: m
%
% INPUT
%   colony: 4*n matrix
%           Each column indicates a voxel
%           1st, 2nd, and 3rd rows indicate xyz subscripts of the voxel
%           4th row indicates the number of matching voxels around the
%           voxel or 1 if the voxel is sub-graspable.
%           This colony is bigger than mesh threshold in either x or y-direction
%   number_of_voxels_in_Colony : n
%   MESH_SIZE : length of one side of a mesh in real world [mm]
%   VOXEL_SIZE : length of one side of voxel [m]
%
%--------------------------------------------------------------------------
function [target_list_of_colony,number_of_targets_in_colony] = colony_mesh(colony,number_of_voxels_in_colony,mesh_size,voxel_size)

% Change the unit of MESH_SIZE from [m] to [], voxel dimension.
mesh_dimension = mesh_size / voxel_size;

% Prepare for the loop.
% Measure dimensions of the colony and decide how to cut the colony based
% on the centroid of the colony.
% This is because we want to choose the targets discretely.
centroid_of_colony = mean(colony(1:2,:),2);
x_min = min(colony(1,:));
y_min = min(colony(2,:));
x_MAX = max(colony(1,:));
y_MAX = max(colony(2,:));
number_of_meshes_in_X_negative_direction = ceil((centroid_of_colony(1)-x_min)/mesh_dimension);
number_of_meshes_in_Y_negative_direction = ceil((centroid_of_colony(2)-y_min)/mesh_dimension);
number_of_meshes_in_X_positive_direction = ceil((x_MAX - centroid_of_colony(1))/mesh_dimension);
numberOf_meshes_in_Y_positive_direction = ceil((y_MAX - centroid_of_colony(2))/mesh_dimension);

number_of_meshes_in_X_direction = number_of_meshes_in_X_negative_direction + number_of_meshes_in_X_positive_direction;
number_of_meshes_in_Y_direction = number_of_meshes_in_Y_negative_direction + numberOf_meshes_in_Y_positive_direction; 

% Set positions of a first mesh
x_min_in_mesh = centroid_of_colony(1) - (number_of_meshes_in_X_negative_direction * mesh_dimension);
y_min_in_mesh = centroid_of_colony(2) - (number_of_meshes_in_Y_negative_direction * mesh_dimension);

% Prepare arrays
voxels_in_mesh = zeros(4,number_of_voxels_in_colony);
target_list_of_colony = zeros(4,number_of_voxels_in_colony);


% Find the highest voxel as a representative voxel.
z_Maximum = max(colony(3,:));
voxels_in_z_maximum = zeros(4,number_of_voxels_in_colony);
for index_of_voxel_in_colony = 1:number_of_voxels_in_colony
    if colony(3,index_of_voxel_in_colony) == z_Maximum
        voxels_in_z_maximum(:,index_of_voxel_in_colony) = colony(:,index_of_voxel_in_colony);
    end
end
[~,idx] = max(voxels_in_z_maximum(4,:));
main_target = voxels_in_z_maximum(:,idx);

% Add the voxel in a target list 
target_list_of_colony(:,1) = voxels_in_z_maximum(:,idx);
number_of_targets_in_colony = 1;

% Calculate centroids in each mesh as sub-representative points
for x_index_of_mesh = 1: number_of_meshes_in_X_direction
    for y_index_of_mesh = 1: number_of_meshes_in_Y_direction
        x_MAX_in_mesh = x_min_in_mesh + mesh_dimension;
        y_MAX_in_mesh = y_min_in_mesh + mesh_dimension;
        % We do not think sub-representative points as centroids in the
        % mesh which has the representative voxel.
        if (x_min_in_mesh <= main_target(1)) && (main_target(1) <= x_MAX_in_mesh) && (y_min_in_mesh <=  main_target(2)) && (main_target(2)<= y_MAX_in_mesh)
            break
        else
            % Find the voxels in the mesh and add to the list of
            % voxels_in_mesh
            for index_of_voxel_in_colony = 1:number_of_voxels_in_colony
                if (x_min_in_mesh <= colony(1,index_of_voxel_in_colony)) && (colony(1,index_of_voxel_in_colony) <= x_MAX_in_mesh) && (y_min_in_mesh <=  colony(2,index_of_voxel_in_colony)) && (colony(2,index_of_voxel_in_colony) <= y_MAX_in_mesh)
                    voxels_in_mesh(:,index_of_voxel_in_colony) = colony(:,index_of_voxel_in_colony);
                end
            end
            % Delete 0 columns
            voxels_in_mesh = voxels_in_mesh(:,any(voxels_in_mesh,1));
            
            % If voxels_in_mesh is not empty, calculate the centroid of the
            % voxels inside of the mesh, and add the target list
            if isempty(voxels_in_mesh) == 0
                centroid_of_voxels_in_mesh = mean(voxels_in_mesh,2);
                number_of_targets_in_colony = number_of_targets_in_colony + 1;
                target_list_of_colony(:,number_of_targets_in_colony) = centroid_of_voxels_in_mesh';
            end
            % Reset the list of voxels in mesh
            voxels_in_mesh = zeros(4,number_of_voxels_in_colony);
            
            % Move to the next mesh in the y-direction 
            y_min_in_mesh = y_min_in_mesh + mesh_dimension;
        end
    end
    % Go back to the first position in the y-direction
    y_min_in_mesh = centroid_of_colony(2) - (number_of_meshes_in_Y_negative_direction * mesh_dimension);
    % Move to the next layer in the x-direction
    x_min_in_mesh = x_min_in_mesh + mesh_dimension;
end

% Delete 0 columns in the target list
target_list_of_colony = target_list_of_colony(:,any(target_list_of_colony,1));
end