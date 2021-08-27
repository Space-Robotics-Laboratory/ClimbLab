%%%%%% Configuration
%%%%%% config_sensing_camera_param
%%%%%% 
%%%%%% Configure default RealSense Camera parameters
%%%%%% 
%%%%%% Created 2021-02-10
%%%%%% Keigo Haji
%%%%%% Last update: 2021-02-25
%%%%%% Keigo Haji
%
%
% Load default configurations for a sensing camera
%
% Function variables:
%
%     OUTPUT
%         sensing_camera_param
%     INPUT
%         -

function sensing_camera_param = config_sensing_camera_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Sensing Camera on/off 
sensing_camera_param.sensing_flag =  'off';
%%% Type of Sensing
sensing_camera_param.sensing_type = 'RealSense_d435i';   %'RealSense_d435i' or 'circle'

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Position settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Following Values are based on the RealSense D435i on HubRobo base.
sensing_camera_param.mounting_angle =  [0; -45; 0]*pi/180;  % rpy
sensing_camera_param.mounting_position = [0.079; -0.011 ; 0.07];    %[m] Mesured by CAD of Hubrobo v3.2

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% fov settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Following Values are written in RealSense D435i datasheet.
sensing_camera_param.fov_horizontal = 86*pi/180;     %[radian]
sensing_camera_param.fov_vertical = 57*pi/180;   %[radian]
sensing_camera_param.fov_max_distance = 2;   %[m]
sensing_camera_param.fov_min_distance = 0.28;    %[m]

%%% Following Values are set for circular sensing.
sensing_camera_param.circular_radius_from_base_pos = 0.4; %Radius to be sensed from the base center of the robot

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initial Known area settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Known area shape
sensing_camera_param.known_area_shape = 'rectangle';   %'rectangle' or 'circle'

%%% Rectangle Param
    % Set the range of acquisition in the +x and +-y directions from the initial robot position. 
    sensing_camera_param.ini_margin_from_base_pos_to_plus_x = 0.4;  %[m]
    sensing_camera_param.ini_margin_from_base_pos_to_y = 0.4;    %[m]

%%% Circle Param
sensing_camera_param.ini_circular_radius_from_base_pos = 0.4;    %[m]

end