%%%%%% Initialize
%%%%%% ini_surface
%%%%%% 
%%%%%% Initialize surface parameters
%%%%%% 
%%%%%% Created 2020-04-06
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-06-28
%%%%%% Keigo Haji
%
%
% Load surface points from .mat file and set contact characteristics (stiffness and damping)
%
% Function variables:
%
%     OUTPUT
%         surface_param.min   : Minimum value of surface height [m] (scalar)
%         surface_param.K     : Ground reaction force stiffness coefficient (scalar)
%         surface_param.D     : Ground reaction force damping coefficient (scalar)
%     INPUT
%         environment_param   : Parameters for environment (class) 
%
% [NOTE]
% If you added new bouldering holds map taken by RealSense camera, you must
% include 'climbing_holds' in the .mat file name. This is because we
% switch how to plot the surface in vis_surface depending on the name of
% the map. 

function surface_param = ini_surface(environment_param)

global x ; global y ; global z;
surface_param.graspable_points = [];
surface_param.known_environment = [];

switch environment_param.surface_type

case 'flat_HR'
    load('map_flat_HR.mat');
    
case 'flat_HR_5m_x_5m'
    load('map_flat_HR_5m_x_5m.mat');
    
case 'flat_zero-height_5m_x_5m'
    load('map_flat_zero-height_5m_x_5m.mat');

case 'rough'
	load('map_uneven.mat');

case 'rough_3m_x_3m'
	load('map_uneven_3m_x_3m.mat');
    
case 'climbing_holds_map_on_testfield'
    load('climbing_holds_map_on_testfield.mat'); 
    % see also ini_graspable_points.m
    
case 'climbing_holds_map_on_full_testfield'
    load('climbing_holds_map_on_full_testfield.mat'); 
    % see also ini_graspable_points.m   
    
case 'climbing_holds_1m_x_1m'
    load('climbing_holds_1m_x_1m.mat'); 
    % see also ini_graspable_points.m
    
case 'flat_003' 
	load('flat_delta_003.mat');
    
case 'flat_006' 
	load('flat_delta_006.mat');
    
case 'flat_008' 
	load('flat_delta_008.mat');
    
case 'flat_009' 
	load('flat_delta_009.mat');
    
case 'flat_012' 
	load('flat_delta_012.mat');
    
case 'grid_3mx3m_dx100mm_thined_30' 
	load('map_grid_3mx3m_dx100mm_thined_30.mat');
    
case 'grid_3mx3m_dx100mm_thined_40' 
	load('map_grid_3mx3m_dx100mm_thined_40.mat');
    
case 'grid_3mx3m_dx100mm_thined_50' 
	load('map_grid_3mx3m_dx100mm_thined_50.mat');
    
case 'grid_3mx3m_dx100mm_thined_50_ver2' 
	load('map_grid_3mx3m_dx100mm_thined_50_ver2.mat');

case 'test_grid_map_3mx3m_dx85mm'
    load('test_grid_map_3mx3m_dx85mm.mat');    
    surface_param.known_environment = z;
    
case 'test_grid_map_3mx3m_dx85mm_1_void_area_at_center_25cm'
    load('test_grid_map_3mx3m_dx85mm_1_void_area_at_center_25cm.mat');

case 'test_grid_map_3mx3m_dx85mm_1_void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_1_void_area_at_center_40cm.mat');

case 'test_grid_map_3mx3m_dx85mm_thined_20_1void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_thined_20_1void_area_at_center_40cm.mat');

case 'test_grid_map_3mx3m_dx85mm_thined_15_1void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_thined_15_1void_area_at_center_40cm.mat');

case 'test_grid_map_3mx3m_dx85mm_thined_15_1void_area_at_center_40cm_ver2'
    load('test_grid_map_3mx3m_dx85mm_thined_15_1void_area_at_center_40cm_ver2.mat');

case 'test_grid_map_3mx3m_dx85mm_thined_30_1void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_thined_30_1void_area_at_center_40cm.mat');
    
case 'test_grid_map_3mx3m_dx70mm_thined_50_1void_area_40cm_at_center'
    load('test_grid_map_3mx3m_dx70mm_thined_50_1void_area_40cm_at_center.mat');
    
case 'test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm.mat');
    
case 'test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm_ver2'
    load('test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm_ver2.mat');
    
case 'test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm_ver3'
    load('test_grid_map_3mx3m_dx85mm_thined_40_1void_area_at_center_40cm_ver3.mat');
    
case 'test_grid_map_3mx3m_dx85mm_thined_50'
    load('test_grid_map_3mx3m_dx85mm_thined_50.mat');    
        
case 'test_grid_map_3mx3m_dx85mm_thined_50_1void_area_at_center_40cm'
    load('test_grid_map_3mx3m_dx85mm_thined_50_1void_area_at_center_40cm.mat');    
    
case 'test_grid_map_3mx3m_dx85mm_thined_50_1void_area_at_center_40cm_ver2'
    load('test_grid_map_3mx3m_dx85mm_thined_50_1void_area_at_center_40cm_ver2.mat');    
    
case 'test_grid_map_3mx3m_dx85mm_2void_areas'
    %%% Get the Obstacles infromation
    load('known_test_grid_map_3mx3m_dx85mm_2void_areas_with_15cm_margin.mat');
    surface_param.known_environment = z;
    load('test_grid_map_3mx3m_dx85mm_2void_areas.mat')

case 'test_grid_map_3mx3m_dx85mm_thined_20_2void_areas'
%     %%% Get the Obstacles infromation
%     surface_param.known_environment =  load('test_grid_map_3mx3m_dx85mm_2void_areas.mat');
%     surface_param.known_environment.x = x;
%     surface_param.known_environment.y = y;
%     surface_param.known_environment.z = z;    
    load('test_grid_map_3mx3m_dx85mm_thined_20_2void_areas.mat');
    
case 'test_grid_map_3mx3m_dx85mm_thined_30_2void_areas'
    %%% Get the Obstacles infromation
    load('test_grid_map_3mx3m_dx85mm_thined_30_2void_areas.mat');
    
case 'test_grid_map_3mx3m_dx50mm_thined_70_2void_areas'
    load('test_grid_map_3mx3m_dx50mm_thined_70_2void_areas.mat');
    
case 'test_grid_map_3mx3m_dx50mm_thined_70'
    load('test_grid_map_3mx3m_dx50mm_thined_70.mat');

case 'test_grid_map_3mx3m_dx10mm_thined_0988'
    load('test_grid_map_3mx3m_dx10mm_thined_0988.mat');    
    
otherwise
	disp('invalid surface');

end

surface_param.min = min(min(z));
surface_param.max = max(max(z));

% Contact parameters
surface_param.K = environment_param.surface_K;  % Floor reaction force stiffness coefficient
surface_param.D = environment_param.surface_D;  % Floor reaction force damping coefficient
