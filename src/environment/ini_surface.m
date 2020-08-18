%%%%%% Initialize
%%%%%% ini_surface
%%%%%% 
%%%%%% Initialize surface parameters
%%%%%% 
%%%%%% Created 2020-04-06
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-06
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
%         surface_type        : Surface type (string: flat_HR, rough) 

function surface_param = ini_surface(environment_param)

global x ; global y ; global z;

switch environment_param.surface_type

case 'flat_HR'
    load('map_flat_HR.mat');

case 'rough'
	load('map_uneven.mat');

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

otherwise
	disp('invalid surface');

end

surface_param.min = min(min(z));

% Contact parameters
surface_param.K = environment_param.surface_K;  % Floor reaction force stiffness coefficient
surface_param.D = environment_param.surface_D;  % Floor reaction force damping coefficient
