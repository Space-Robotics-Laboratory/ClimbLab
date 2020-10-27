%%%%%% Update
%%%%%% upd_motion_planning
%%%%%% 
%%%%%% Update motion variables
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Update motion planning variables: desired trajectories, current step positions and state variable
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param  : Parameters for motion planning (class)
%         des_SV      			 : Desired state variables (SpaceDyn class)
%     INPUT
%         motion_planning_param  : Parameters for motion planning (class)
%         gait_param             : Parameters for gait (class)
%         path_planning_param    : Parameters for path planning (class)
%		  LP                     : Link parameters (SpaceDyn class)
%         SV 	      			 : State variables (SpaceDyn class)
%         des_SV      			 : Desired state variables (SpaceDyn class)
% 		  cont_POS				 : Contact position [m] (3xn matrix)
%         time     				 : Simulation time [s] (scalar)


function [motion_planning_param, des_SV] = upd_motion_planning(motion_planning_param, gait_param, path_planning_param, LP, SV, des_SV, cont_POS, time)


switch gait_param.type

case 'crawl_fixed_stride'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, path_planning_param] = upd_swing_traject_spline3_two_segments(path_planning_param,gait_param,motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, path_planning_param] = upd_base_traject_spline3_one_segment(path_planning_param, gait_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(path_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(path_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_param, path_planning_param, des_SV, time);
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);

case 'do_nothing'
	des_SV = des_SV;

case 'crawl_uno_ver'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, path_planning_param] = upd_swing_traject_spline3_two_segments(path_planning_param,gait_param,motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, path_planning_param] = upd_base_traject_spline3_one_segment(path_planning_param, gait_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(path_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(path_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_param, path_planning_param, des_SV, time);
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
    
% for now, the below is exact same as the case of 'crawl_uno_ver' (20200703)
case 'nonperiodic_uno_ver'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, path_planning_param] = upd_swing_traject_spline3_two_segments(path_planning_param,gait_param,motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, path_planning_param] = upd_base_traject_spline3_one_segment(path_planning_param, gait_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(path_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(path_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_param, path_planning_param, des_SV, time);
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);

otherwise
	
	disp('Invalid gait type')

end

end