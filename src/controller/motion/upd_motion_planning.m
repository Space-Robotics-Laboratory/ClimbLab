%%%%%% Update
%%%%%% upd_motion_planning
%%%%%% 
%%%%%% Update motion variables
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-04-29
%%%%%% Keigo Haji
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
%         gait_planning_param    : Parameters for gait planning (class)
%		  LP                     : Link parameters (SpaceDyn class)
%         SV 	      			 : State variables (SpaceDyn class)
%         des_SV      			 : Desired state variables (SpaceDyn class)
% 		  cont_POS				 : Contact position [m] (3xn matrix)
%         time     				 : Simulation time [s] (scalar)


function [motion_planning_param, des_SV] = upd_motion_planning(motion_planning_param, gait_planning_param, LP, SV, des_SV, cont_POS, POS_e, time)

switch gait_planning_param.type

case 'periodic_crawl_gait'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments_time(gait_planning_param, motion_planning_param, time, LP);
	% Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment_time(gait_planning_param, motion_planning_param, time, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments_time(gait_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	%%% Update desired state variables
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);

case 'crawl_fixed_stride'
    
    % If the waking sequence is changed, the trajectory generation of the
    % swing limb is skipped, because it is not made into a swing leg. 
    if gait_planning_param.crawl_gait_sequence_change_flag == false
        %%% Update trajectory
        % Define new trajectory for swing leg
        [motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
        %%% Update desired motion for current time-step
        % Desired position for legs for the current step
        motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(gait_planning_param, motion_planning_param, LP, cont_POS, time);
    end
    
    %%% Update trajectory
    % Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
    %%% Update desired motion for current time-step
    % Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);

case 'diagonal_gait_fixed_stride'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(gait_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);

case 'do_nothing'
	des_SV = des_SV;

case 'crawl_gait_for_discrete_footholds'
    
    % If the waking sequence is changed, the trajectory generation of the
    % swing limb is skipped, because it is not made into a swing leg.
    if gait_planning_param.crawl_gait_sequence_change_flag == false
        %%% Update trajectory
        % Define new trajectory for swing leg
        [motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
        %%% Update desired motion for current time-step
        % Desired position for legs for the current step
        motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(gait_planning_param, motion_planning_param, LP, cont_POS, time);
    end
    
    %%% Update trajectory
    % Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
	%%% Update desired motion for current time-step
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);
    
case 'diagonal_gait_for_discrete_footholds'    
    %%% Update trajectory
    % Define new trajectory for swing leg
    [motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
    %%% Update desired motion for current time-step
    % Desired position for legs for the current step
    motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(gait_planning_param, motion_planning_param, LP, cont_POS, time);
    
    %%% Update trajectory
    % Define new trajectory for base
    [motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
    %%% Update desired motion for current time-step
    % Desired position for base for the current step
    motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
    
    %%% Update desired state variables
    % Inverse Kinematics
    des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);
    
case 'nonperiodic_gait_for_discrete_footholds'
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline3_two_segments(gait_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
	%%% Update desired state variables
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);

case 'GIA_opt_based_pose_planner' % This is under development and does not work in the current version
	%%% Update trajectory
	% Define new trajectory for swing leg
	[motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments(gait_planning_param, motion_planning_param, des_SV, LP);
	% Define new trajectory for base
	[motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP);
	
	%%% Update desired motion for current time-step
	% Desired position for legs for the current step
	motion_planning_param = upd_swing_current_des_POS_spline_for_intermittent_gait(gait_planning_param, motion_planning_param, LP, cont_POS, time);
	% Desired position for base for the current step 
	motion_planning_param = upd_base_current_des_POS_spline3_one_segment(gait_planning_param, motion_planning_param, LP, time);
	
    des_SV = upd_base_ori(des_SV, gait_planning_param, LP, time);
	%%% Update desired state variables
	% Inverse Kinematics 
	des_SV = upd_inverse_kinematics(motion_planning_param, LP, des_SV);
	% Desired gripper state
	des_SV = upd_gripper_state_time(gait_planning_param, LP, des_SV, time);
otherwise
	
	disp('Invalid gait type')

end


%%% update four limb's trajectories
motion_planning_param.trajectories.LF = [motion_planning_param.trajectories.LF POS_e(:,1)];
motion_planning_param.trajectories.LH = [motion_planning_param.trajectories.LH POS_e(:,2)];
motion_planning_param.trajectories.RH = [motion_planning_param.trajectories.RH POS_e(:,3)];
motion_planning_param.trajectories.RF = [motion_planning_param.trajectories.RF POS_e(:,4)]; 



end