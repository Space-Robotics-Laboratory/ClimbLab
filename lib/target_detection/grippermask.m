%%%%%% Gripper-Mask
%%%%%% grippermask.m
%%%%%% 
%%%%%% Make the gripper-mask
%%%%%% 
%%%%%% Created 2020-08-17
%%%%%% Keigo Haji
%%%%%% Last update: 2020-10-01
%%%%%% Keigo Haji
%--------------------------------------------------------------------------
% grippermask.m makes the gripper_mask, which is the 3-dimensional array 
% composed of 0 and 1 considering geometric parameters of the gripper.
%
% Function variables:
% 
% OUTPUT
%   gripper_mask : n*n*m array composed of 0 and 1. 
%
% INPUT
%   gripper_param : geometric parameters of the gripper which are set in
%                   the config file.
%   voxel_size : length of one side of voxel [m] 
%   gripper_or_peaks : "gripper" or "peaks"
%                      If the string is "gripper", the gripper_mask is made
%                      based on the mechanical shape of the gripper.
%                      If the string is "peaks", the girpper_mask is made
%                      just 
% 
%%% Note %%%
% A solid element is 1 and an empty element is 0.
% The area composed of 1 is the graspable area by the gripper.
% The area composed of 0 is the un-graspable area by the gripper.
%--------------------------------------------------------------------------
function [gripper_mask] = grippermask(gripper_param, voxel_size, gripper_or_peaks)

% Calculate the ratio of voxel size and 1mm in order to keep the gripper size in real world regardless of voxel size
ratio = 1/(voxel_size*1000);

if gripper_or_peaks == "gripper"
    % Reduce or magnify the gripper's parameters to fit voxel's dimension
    % Change demensions from [mm] to [voxels]
    palm_diameter = round(gripper_param.palm_diameter*ratio);
    palm_diameter_of_finger_joints = round(gripper_param.palm_diameter_of_finger_joints*ratio);
    finger_length = round(gripper_param.finger_length*ratio);
    spine_length = round(gripper_param.spine_length*ratio);
    spine_depth = round(gripper_param.spine_depth*ratio);
    opening_spine_radius = round(gripper_param.opening_spine_radius*ratio);
    opening_spine_depth = round(gripper_param.opening_spine_depth*ratio);
    closing_height = round(gripper_param.closing_height*ratio);
    margin_of_top_solid_diameter = round(gripper_param.margin_of_top_solid_diameter*ratio);
    inside_margin_of_bottom_void_diameter = round(gripper_param.inside_margin_of_bottom_void_diameter*ratio);


    % Set the gripper-mask size
    gripper_mask_half_size = (palm_diameter_of_finger_joints/2) + finger_length + spine_length;
    gripper_mask_size = 2*gripper_mask_half_size + 1;   % +1 makes the mask size odd
    gripper_mask_height = closing_height;


    % Calculate the parameters to determine solid area and void area
    gripper_mask_top_solid_radius = round(( palm_diameter + margin_of_top_solid_diameter)/2);
    gripper_mask_clearance = round((gripper_mask_size - palm_diameter)/2 * tand(90 - gripper_param.opening_angle));
    gripper_mask_bottom_void_radius = round(palm_diameter/2 + (gripper_mask_height * tand(gripper_param.closing_angle)) - inside_margin_of_bottom_void_diameter);


    % Prepare a 3-dimensional array composed of 0
    gripper_mask = zeros(gripper_mask_size, gripper_mask_size, gripper_mask_height);


    % Make the gripper_mask by setting the elements of 1.
    for z_subscript = 1:gripper_mask_height
        % Calculate radius of inner cone and outer solid area.
        grippable_radius = gripper_mask_top_solid_radius + (gripper_mask_half_size - gripper_mask_top_solid_radius)*(z_subscript - 1)/(gripper_mask_clearance  - 1);
        unreachble_radius = gripper_mask_half_size - round(gripper_mask_half_size  -(opening_spine_radius + spine_depth)) * (z_subscript - 1)/(opening_spine_depth - 1);
        for y_subscript = 1:gripper_mask_size
            for x_subscript = 1:gripper_mask_size
                % Caculate the distance from center of layer  
                distance_from_center_of_layer = sqrt((gripper_mask_half_size+1 - x_subscript)^2+(gripper_mask_half_size+1 - y_subscript)^2);
                % Judges whether it is a solid(1) region or not.
                if  (z_subscript <= gripper_mask_clearance  && distance_from_center_of_layer <  grippable_radius||...
                     z_subscript <= gripper_mask_clearance && distance_from_center_of_layer >  unreachble_radius ||...
                     z_subscript > gripper_mask_clearance  && z_subscript ~= gripper_mask_height ||...
                     z_subscript == gripper_mask_height && distance_from_center_of_layer > gripper_mask_bottom_void_radius)
                    % Set the element as 1
                    gripper_mask(x_subscript,y_subscript,z_subscript) = 1;
                end
            end
        end
    end
    
    
elseif gripper_or_peaks == "peaks"
    % Reduce or magnify the gripper's parameters to fit voxel's dimension
    % Change demensions from [mm] to [voxels]
    gripper_mask_half_size = round(gripper_param.half_size_of_gripper_mask_for_peaks * ratio);
    gripper_mask_height = round(gripper_param.height_of_gripper_mask_for_peaks * ratio);
    
    % Set the gripper-mask size
    gripper_mask_size = 2*gripper_mask_half_size + 1;
    
    % Make a 3-dimensional array composed of 1
    gripper_mask = ones(gripper_mask_size, gripper_mask_size, gripper_mask_height);
    
end   
    
    
end

