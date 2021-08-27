%%%%%% Initialize
%%%%%% ini_gait
%%%%%% 
%%%%%% Initialize gait planning parameters
%%%%%% 
%%%%%% Created: 2020-04-08
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2021-06-28
%%%%%% Keigo Haji
%
% Initialize parameters for gait, based on the selected gait type
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param    : Parameters for gait planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%     INPUT
%         SV                    : State values (SpaceDyn class)

function [gait_planning_param, motion_planning_param] = ini_gait(gait_planning_param, SV)


motion_planning_param = [];
% initialize the limb trajectories logging arrays
motion_planning_param.trajectories.LF = []; 
motion_planning_param.trajectories.LH = [];
motion_planning_param.trajectories.RH = [];
motion_planning_param.trajectories.RF = [];

%%% Initialize goals' count as passing points
gait_planning_param.goal_num = 1;


%%% Set the Walking sequence for crawl gait based on the moving direction
% The sequences can be distinguished by the first swing limb, which is
% designated by a different number. 
if contains(gait_planning_param.type, 'crawl') || contains(gait_planning_param.type, 'diagonal')
    if isempty(gait_planning_param.sequence)
        gait_planning_param.crawl_gait_direction_change = 'on';
        vec_bg = gait_planning_param.goal(1:3, gait_planning_param.goal_num) - SV.R0;
        % Calculate cross product to decide the direction which robot goes
        cross_vec_tmp1 = cross([1,1,0],[vec_bg(1),vec_bg(2),0]);
        if cross_vec_tmp1(3) < 0
            cross_vec_tmp2 = cross([1,-1,0],[vec_bg(1),vec_bg(2),0]);
            if cross_vec_tmp2(3) < 0
                % Limb 1&2 are hind legs
                gait_planning_param.sequence = [1; 4; 2; 3];
            elseif cross_vec_tmp2(3) >= 0
                % Limb 2&3 are hind legs
                gait_planning_param.sequence = [2; 1; 3; 4];
            end
        elseif cross_vec_tmp1(3) >= 0 
        cross_vec_tmp2 = cross([1,-1,0],[vec_bg(1),vec_bg(2),0]);            
        if cross_vec_tmp2(3) < 0
            % Limb 1&4 are hind legs
            gait_planning_param.sequence = [4; 3; 1; 2];
        elseif cross_vec_tmp2(3) >= 0
            % Limb 3&4 are hind legs
            gait_planning_param.sequence = [3; 2; 4; 1];
        end
        end
    else
        gait_planning_param.crawl_gait_direction_change = 'off';
        gait_planning_param.crawl_gait_sequence_change_flag = false;
    end
else
    ;
end

switch gait_planning_param.type

case 'do_nothing'

case 'crawl_fixed_stride' 

	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
	    gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end

case 'periodic_crawl_gait'

	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
	    gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end

case 'diagonal_gait_fixed_stride' %%% the followings are same as 'crawl_fixed_stride' 
	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
	    gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end

case 'crawl_gait_for_discrete_footholds'

	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
		gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end

case 'diagonal_gait_for_discrete_footholds'

	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
		gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end
    
% for now, the below is exact same as the case of 'crawl_uno_ver' (20200703)
case 'nonperiodic_gait_for_discrete_footholds'

    % Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

% 	% Swing phase for each leg [s]
% 	phi = [0;
%        gait_planning_param.T_d   + gait_planning_param.T_ddd;
%        2*gait_planning_param.T_d + gait_planning_param.T_ddd;
%        3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

% 	% Swing phase for each limb based on the sequence
% 	for i=1:4
% 		gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
%     end
    
case 'GIA_opt_based_pose_planner' % This is under development and does not work in the current version

	% Swing period for one leg
	gait_planning_param.T_d   = (1 - gait_planning_param.beta)*gait_planning_param.T;
	% Half swing period
	gait_planning_param.T_dr  = gait_planning_param.T_d/2;
	% Supporting period for one leg
	gait_planning_param.T_dd  = gait_planning_param.beta*gait_planning_param.T; 
	% All legs supporting period
	gait_planning_param.T_ddd = (2*gait_planning_param.beta - 1.5)*gait_planning_param.T;

	% Swing phase for each leg [s]
	phi = [0;
       gait_planning_param.T_d   + gait_planning_param.T_ddd;
       2*gait_planning_param.T_d + gait_planning_param.T_ddd;
       3*gait_planning_param.T_d + 2*gait_planning_param.T_ddd;]';

	% Swing phase for each limb based on the sequence
	for i=1:4
		gait_planning_param.phi(gait_planning_param.sequence(i)) = phi(i);
    end
    gait_planning_param.base_move = false;
    
otherwise 
	disp('Invalid gait type')

end

% Initialize the base orientation value for gait planning
gait_planning_param.base_orientation = SV.Q0;
gait_planning_param.base_orientation_next = SV.Q0;

% Initialize the valuables to record footprint history 
gait_planning_param.footholds_history_limb = [];
gait_planning_param.footholds_count_limb = ones(4,1);

% Initialize the valuables to record com projection
% @TODO: make a array to record the CoM projection 
gait_planning_param.com_projection_history = [];

% Initialize the valuables to record swing number
gait_planning_param.swing_number = []; 
gait_planning_param.swing_number_history = [];


end