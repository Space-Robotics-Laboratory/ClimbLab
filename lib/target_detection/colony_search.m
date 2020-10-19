%%%%%% Find Colonies
%%%%%% colony_search.m
%%%%%% 
%%%%%% Narrow down the candidates from the graspable points and select them discretely. 
%%%%%% 
%%%%%% Created 2020-09-22
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-12
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% colony_search.m selects the candidates of targets for gripping from the matching list.
% First, find "colonies" by using the neighbor search algorithm.
% Second, select a representative point in each colony which is located in
% the highest position and has higher matching voxels.
% Finally, if the colony is bigger than MESH_THRESHOLD, section the 
% colony into meshes and select a sub-representative point in each mesh.
%
% Function variables:
%
% OUTPUT
%   target_list: 4*m matrix
%                the candidates for gripping chosen discretely in the colonies
%
% INPUT
%   voxel_coordinates_of_graspable_points: 4*n matrix
%                                          Each column indicates a matched voxel. 
%                                          1st, 2nd, and 3rd raws indicate x y z subs of voxels.
%                                          4th raw indicates the number of solid voxels, or 1 if the voxel is sub-graspable.
%   matching_settings.    
%       COLONY_THRESHOLD: a threshold of closeness between voxels as a colony []
%       MESH_THRESHOLD: a threshold of size of a colony [m]
%                       If a colony is bigger than mesh_thr either in x-dircetion
%                       or y-direction, the colony is sectionized 
%   grid_size: lenght of one side of the grid which is defined in map interpolation [m]
%   VOXEL_SIZE: length of one side of voxel [m] 
%--------------------------------------------------------------------------
function [target_list] = colony_search(voxel_coordinates_of_graspable_points, matching_settings, voxel_size, grid_size)
% Set the colony_threshold
% If solid voxels are located nearer than colony_threshold, the voxels are considered as a colony
% When interpolation setting is "on", the density of the solid voxels is uniform.
% When interpolation setting is "off", the density of solid voxels varies
% from place to place, and grid_size is not defined. So, we define the
% colony_threshold by the number of voxels you think are close.
if nargin == 3
    colony_threshold = matching_settings.colony_threshold;   %[] voxels
elseif nargin == 4
    colony_threshold = ceil(grid_size /  voxel_size);
end

% If the input matrix is 0, it outputs a 0 matrix
if isempty(voxel_coordinates_of_graspable_points) == 1
    target_list = zeros(4,0);
else
    
    % Change the unit of MESH_THRESHOLD from [mm] to [], voxel dimension.
    mesh_threshold = matching_settings.mesh_threshold / voxel_size;
    
    % Prepare for the loop
    [~,number_of_match_voxels] = size(voxel_coordinates_of_graspable_points);
    remaining_match_voxels = voxel_coordinates_of_graspable_points;
    target_list = zeros(4,number_of_match_voxels);
    number_of_targets = 0;
    
    
    %=== Find a colony ====================================================
    while 1
        % Make a colony as a matrix of 0
        [~,number_of_remaining_voxels] = size(remaining_match_voxels);
        colony = zeros(4,number_of_remaining_voxels);
        
        % Set the first voxel of the remaining voxels' list as the first
        % element of the colony.
        first_voxel_of_colony = remaining_match_voxels(:,1);
        colony(:,1)= first_voxel_of_colony;
        
        % Delete the first voxel in a colony from a list of remaining
        % voxels.
        remaining_match_voxels(:,1) = [];
        
        % If all the matching voxels are classified into colonies, the loop
        % ends.
        if isempty(remaining_match_voxels) == 1
            number_of_targets = number_of_targets + 1;
            target_list(:,number_of_targets) = first_voxel_of_colony;
            break
        end
        
        % Reduce the number of remaining voxels
        number_of_remaining_voxels = number_of_remaining_voxels - 1;
        
        % Prepare for the loop
        index_list_of_voxels_in_colony = zeros(1,number_of_remaining_voxels);
        number_of_voxels_in_colony = 1;
        
        % Set the reference voxel
        % We search for other voxels around this reference voxel
        index_of_reference_voxel = 1;
        referece_voxel_of_colony = colony(:,index_of_reference_voxel);
                
        % Find the voxels located near the reference voxel by using a
        % neighbor search algorithm.
        % Check the other voxels around the reference voxels until no
        % voxels can be found nearby in all voxels in the colony.
        % Make a list of the voxels in the colony and add them to the
        % colony.
        while 1
            for index_of_voxel = 1 : number_of_remaining_voxels
                
                % Calculate distances between the reference voxel and the
                % other remaining voxels in x- and y-direction.
                distance_in_x_direction = abs(referece_voxel_of_colony(1) - remaining_match_voxels(1,index_of_voxel));
                distance_in_y_direction = abs(referece_voxel_of_colony(2) - remaining_match_voxels(2,index_of_voxel));
                
                % If a voxel is located nearer than COLONY_THRESHOLD, the
                % voxel is considered as an element of the colony.
                if (distance_in_x_direction <= colony_threshold) && (distance_in_y_direction <= colony_threshold)
                    colony(:,number_of_voxels_in_colony+1) = remaining_match_voxels(:,index_of_voxel);
                    index_list_of_voxels_in_colony(1,index_of_voxel) = index_of_voxel;
                    number_of_voxels_in_colony = number_of_voxels_in_colony + 1;
                end
            end
            
            % Delete 0 columns
            index_list_of_voxels_in_colony = index_list_of_voxels_in_colony(:,any(index_list_of_voxels_in_colony,1));
            % Delete voxels classified the colony from the remaining
            % voxels'list
            remaining_match_voxels(:,index_list_of_voxels_in_colony) = [];
            
            % Reset the index list
            number_of_remaining_voxels = size(remaining_match_voxels,2);
            index_list_of_voxels_in_colony = zeros(1,number_of_remaining_voxels);
            
            % Change the reference voxel to the next column's voxel
            index_of_reference_voxel = index_of_reference_voxel+1;
            referece_voxel_of_colony = colony(:,index_of_reference_voxel);
            
            % Check the reference voxel and if it is a 0 vector, we have
            % already considered all voxels in the colony.
            if referece_voxel_of_colony == 0
                break
                % If unsearched voxels do not remain, we finish finding the
                % elements of the colony.
            elseif isempty(remaining_match_voxels) == 1
                break
            end
        end
        
        % Delete 0 columns in the colony matrix
        colony = colony(:,any(colony,1));
        
        
        %=== Select representative points as targets for gripping =============
        
        % Calculate dimensions of the colony in x- and y-direction
        x_dimension_of_colony = max(colony(1,:)) - min(colony(1,:)) + 1;
        y_dimension_of_colony = max(colony(2,:)) - min(colony(2,:)) + 1;
        % If the dimensions are bigger than MESH_THRESHOLD, we section the
        % colony into meshes and consider a representative point in each mesh.
        if (x_dimension_of_colony > mesh_threshold) || (y_dimension_of_colony > mesh_threshold)
            % Use another function of meshcolony
            [target_list_in_colony,number_of_targets_in_colony] = colony_mesh(colony,number_of_voxels_in_colony,matching_settings.mesh_size,voxel_size);
            
            % Add the targets in target list
            for index_list_of_targets = 1:number_of_targets_in_colony
                target_list(:,number_of_targets + index_list_of_targets) = target_list_in_colony(:,index_list_of_targets);
            end
            % Add the number of targets
            number_of_targets = number_of_targets + number_of_targets_in_colony;
            
        else
            % When the colony is small, choose the voxel which is located
            % in the highest position and has higher matching voxels as a representative point.
            z_max = max(colony(3,:));
            voxels_in_z_max = zeros(4,number_of_voxels_in_colony);
            
            for indexOfVoxelInColony = 1:number_of_voxels_in_colony
                if colony(3,indexOfVoxelInColony) == z_max
                    voxels_in_z_max(:,indexOfVoxelInColony) = colony(:,indexOfVoxelInColony);
                end
            end
            [~,indexOfTarget] = max(voxels_in_z_max(4,:));
            
            % Add the number of targets and the target into a target list
            number_of_targets = number_of_targets + 1;
            target_list(:,number_of_targets) = voxels_in_z_max(:,indexOfTarget);
        end
        
        % If all the matching voxels are classified into colonies, the loop
        % ends.
        if isempty(remaining_match_voxels) == 1
            break
        end
        
    end
    
    % Delete 0 columns in the target list
    target_list = target_list(:,any(target_list,1));
    
end
end