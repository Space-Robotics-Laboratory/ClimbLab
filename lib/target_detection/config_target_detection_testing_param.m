%%%%%% Configuration
%%%%%% config_target_detection_testing_param
%%%%%% 
%%%%%% Define target detection parameters for test
%%%%%% 
%%%%%% Created 2021-04-15
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-21
%%%%%% Keigo Haji
%
%
% Load configurations for parameters defined for target detection algorithm 
%
% Function variables:
%
%     OUTPUT
%         gripper_param : parameters for the gripper-mask
%         map_param     : parameters and settings for creating the terrain
%                         matrix
%         matching_settings : settings for matching
%         gripper_or_peaks  : the way of detecting targets, 'gripper' or 'peaks'
%         environment_param : Environment settings
%         ani_settings      ; Animation settings
%     INPUT
%         -
% 

function [gripper_param, map_param, matching_settings, gripper_or_peaks, environment_param, ani_settings] = config_target_detection_testing_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PCD file Name, 'allMediumHolds14', 'testfield_Apr8','rtabmap_Apr22'
map_param.pcd_file_name = 'rtabmap_Apr22.pcd';
environment_param.surface_type = append('climbing_holds_',map_param.pcd_file_name);
gripper_or_peaks = 'gripper';    

% Although the following lines are map_param, these parameters are not used in both main_sim and target_detection.m. 
map_param.delete_lower_points_in_transformed_PCD = 'on';
map_param.delete_lower_points_threshold = -0.01;        %[m]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
environment_param.inc = 0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Surface viz. on/off
ani_settings.surface_show = 'on';
    %%% Surface grid color
    ani_settings.grid_color = [0.9 0.9 0.9];
    %%% Surface color
    ani_settings.surface_color = gray;

%%% Graspable points viz. on/off
ani_settings.graspable_points_show = 'on';
    %%% Graspable points color
    ani_settings.graspable_points_color = [0.0, 0.8, 0.0];
    %%% Graspable points alpha
    ani_settings.graspable_points_alpha = 1.0;
    %%% Graspable points marker
    ani_settings.graspable_points_marker = 'o';
    %%% Graspable points size
    ani_settings.graspable_points_size = 40;

    
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gripper Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% if detection_type is "gripper" %%%%  
gripper_param.palm_diameter = 32;   %[mm]
gripper_param.palm_diameter_of_finger_joints = 28;   %[mm]
gripper_param.finger_length = 15;   %[mm] between joints
gripper_param.spine_length = 15;    %[mm]
gripper_param.spine_depth = 5;      %[mm]
gripper_param.opening_angle = 75;   %[deg]
gripper_param.closing_angle = 30;   %[deg]

% Measured parameters by CAD
gripper_param.opening_spine_radius = 37;      % the distance from the center of palm to the farthest point of the spines
% when gripping 75 degrees shape [mm] (37.4mm)
gripper_param.opening_spine_depth = 5;        % 5.26 [mm]
gripper_param.closing_height = 16;            %[mm] Vertical distance between the tip of the spine and the bottom of the palm when closed

% margins
gripper_param.margin_of_top_solid_diameter = 4;   %[mm]
gripper_param.inside_margin_of_bottom_void_diameter = 2;   %[mm]
    
%%%% if detection_type is "peaks" %%%%
% Gripper-Mask Parameters just for peaks
% Set the size of a rectangle. Determine the search range to detect
% peak.
gripper_param.half_size_of_gripper_mask_for_peaks = 20;     %[mm]
gripper_param.height_of_gripper_mask_for_peaks= 2;          %[mm]


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Map Parameters and settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If the area of the map is larger than a few meters, it may exceed the size limit of the 3D array that MATLAB can handle.
% To avoid this, we can exclude distant points.
map_param.delete_outliers = "off";           % "on" or "off"
map_param.outliers_threshold = 2;            % [m]

map_param.voxel_size = 0.0010;               % length of one side of voxel [m] 
map_param.voxel_threshold = 0.0010;          % gap from center point of voxel [m]
                                             % if the centroid of points in the voxel is farther than this value, the voxel is set 0
map_param.transform = "off";                 % "on" or "off"        
                                             % If you use fractal map or the map which is already transformed into the least-square plane,
                                             % you should set "off"
map_param.interpolation = "off";             % raw data or interpolated map, "on" or "off"
map_param.interpolation_method = "linear";   % method of interpolation, "linear" or "natural"



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Matching Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For a fractal terrain with point clouds at 1mm intervals, set 
% matching_settings.threshold = 1000;
% and set matching_settings.threshold to about 120 
% to use the point clouds obtained by the RealSense camera.
% When you use climbing_holds_map_1m_x_1m_gridformat.mat, set
% matching_settings.threshold = 130;       
% If it is detected in a depression, which is obviously not a convexity,
% this should be resolved by increasing the value. 
matching_settings.threshold = 120;                % number of solid voxels to guarantee the grasp  [] voxels

matching_settings.submatching = "on";             % only regular targets or including sub-targets, "on" or "off"
matching_settings.penalty_coefficient = 100;      % penalty factor for impaired solid voxels  
matching_settings.colony = "on";                  % whether to thin out the graspable points from dense areas, "on" or "off"
matching_settings.colony_threshold = 4;           % the number of vocels you think are close [] voxels
                                                  % This value is only used when map_param.interpolation == "OFF" && matching_settings.colony == "ON";  
matching_settings.mesh_threshold = 0.015;         % mesh threshold [m]
matching_settings.mesh_size = 0.015;              % length of one side of mesh [m]

matching_settings.delete_lower_targets = 'on';
matching_settings.delete_lower_targets_threshold = 0.01;        %[m]



end