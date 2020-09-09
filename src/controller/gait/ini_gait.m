%%%%%% Initialize
%%%%%% ini_gait
%%%%%% 
%%%%%% Initialize gait parameters
%%%%%% 
%%%%%% Created 2020-04-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Initialize parameters for gait, based on the selected gait type
%
% Function variables:
%
%     OUTPUT
%         gait_param             : Parameters for gait (class)
%         path_planning_param    : Parameters for path planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%     INPUT
%         gait_param             : Parameters for gait (class)

function [gait_param, path_planning_param, motion_planning_param] = ini_gait(gait_param)


path_planning_param = [];
motion_planning_param = [];

switch gait_param.type

case 'do_nothing'
	path_planning_param = [];
	motion_planning_param = [];

case 'crawl_fixed_stride'

	% Swing period for one leg
	gait_param.T_d   = (1 - gait_param.beta)*gait_param.T;
	% Half swing period
	gait_param.T_dr  = gait_param.T_d/2;
	% Supporting period for one leg
	gait_param.T_dd  = gait_param.beta*gait_param.T; 
	% All legs supporting period
	gait_param.T_ddd = (2*gait_param.beta - 1.5)*gait_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_param.T_d   + gait_param.T_ddd;
       2*gait_param.T_d + gait_param.T_ddd;
       3*gait_param.T_d + 2*gait_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
	    gait_param.phi(gait_param.sequence(i)) = phi(i);
	end
case 'crawl_uno_ver'

	% Swing period for one leg
	gait_param.T_d   = (1 - gait_param.beta)*gait_param.T;
	% Half swing period
	gait_param.T_dr  = gait_param.T_d/2;
	% Supporting period for one leg
	gait_param.T_dd  = gait_param.beta*gait_param.T; 
	% All legs supporting period
	gait_param.T_ddd = (2*gait_param.beta - 1.5)*gait_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_param.T_d   + gait_param.T_ddd;
       2*gait_param.T_d + gait_param.T_ddd;
       3*gait_param.T_d + 2*gait_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
		gait_param.phi(gait_param.sequence(i)) = phi(i);
    end
    
otherwise 
	disp('Invalid gait type')

end

% Initialize the valuables to record footprint and com projection history 
path_planning_param.footholds_history_limb = [];
path_planning_param.footholds_count_limb = ones(4,1);
%path_planning_param.com_projection_history = [];

end