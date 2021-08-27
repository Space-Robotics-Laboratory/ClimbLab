%%%%%% Update
%%%%%% upd_gait_planning
%%%%%% 
%%%%%% Update gait planning variables
%%%%%% 
%%%%%% Created: 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2021-05-12
%%%%%% Keigo Haji
%
%
% Update the gait planning based on the gait type
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         surface_param         : Parameters for surface (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         SV                    : State values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         environment_param     : Parameters for environment (class)
%         robot_param           : Parameters for robot (class)
%         sensing_camera_param  : Parameters for sensing camera (class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_gait_planning(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, environment_param, robot_param, sensing_camera_param, time)
global d_time;
inc = environment_param.inc;
base_height = robot_param.base_height;
switch gait_planning_param.type

case 'periodic_crawl_gait'
    %%% 1) Moving direction is defined by the config parameter
    %%% 2) Update swing timing of legs and swinging leg number
    gait_planning_param = upd_swing_timing(gait_planning_param, LP, time);
    gait_planning_param = upd_swing_num_periodic_timing(gait_planning_param, LP, time);
    %%% 3) Select the next grasping point
    gait_planning_param = upd_swing_next_pos_periodic_fixed_stride(gait_planning_param, LP, POS_e, time);
	gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
    %%% 4) Select the next position for the base
	gait_planning_param = upd_base_next_pos_crawl_fixed_stride(gait_planning_param, des_SV, SV, LP, time);

case 'crawl_fixed_stride'
    
    %%% 1) Select the new swing leg
	gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time);
    %%% 2) Update walking sequence based on the moving direction
    gait_planning_param = upd_walking_sequence_based_on_moving_direction(gait_planning_param, des_SV, SV, LP, time);
	%%% 3) Select the next grasping point of the selected swing limb
    gait_planning_param = upd_swing_next_pos_crawl_fixed_stride(gait_planning_param, des_SV, SV, LP, POS_e, time);
    %%% 4) Select the next position for the base
    gait_planning_param = upd_base_next_pos_crawl_gait(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, base_height, inc, time);
    %%% 5) Record the footholds of swing leg
    gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
    
case 'do_nothing'

case 'diagonal_gait_fixed_stride'
    
if sum(des_SV.sup) == LP.num_limb    
	%%% 1) Select the new swing leg
	gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time);
	%%% 2) Select the next grasping point of the selected swing limb
	gait_planning_param = upd_swing_next_pos_crawl_fixed_stride(gait_planning_param, des_SV, SV, LP, POS_e, time);
    %%% 3) Record the footholds of swing leg
    gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
	%%% 4) Select the next position for the base
	gait_planning_param = upd_base_next_pos_CoM_projection_on_intersection_of_diagonals(SV,des_SV,LP,gait_planning_param,POS_e,inc,surface_param,base_height,time);
end

case 'crawl_gait_for_discrete_footholds'
    
if sum(des_SV.sup) == LP.num_limb
    %%% 0) update the graspable points in all reachable region of limbs
    gait_planning_param = upd_graspable_points_in_reachable_area(SV,des_SV,LP,surface_param, gait_planning_param, sensing_camera_param);
    
    %%% 1) Select new swing leg
    gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time);
    
    %%% 2) Update Moving Direction (TO BE ADDED)
    
    %%% 3) Update walking sequence based on the moving direction
    gait_planning_param = upd_walking_sequence_based_on_moving_direction(gait_planning_param, des_SV, SV, LP, time);
    
    
    
    %%% Necessary temporary variable declaration @TODO: this should be in
    %%% the function and remove "sum(des_SV.sup) == LP.num_limb" from this
    %%% switch sentence brock.
    % kinematics feasibility chech flag
    gait_planning_param.kinematic_feasibility_check_is_OK = false;
    % Graspable points in Reachable area of each limb @TODO: probably we do
    % not have to prepare this temp variables for each limb by each. --> we
    % can make one single variable for all
    gait_planning_param.LF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,1);
    gait_planning_param.LH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,2);
    gait_planning_param.RH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,3);
    gait_planning_param.RF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,4);
        
    gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride = 0; % Index number used to identify the max value in the vector
    while ~gait_planning_param.kinematic_feasibility_check_is_OK
        %%% 4) Select the next grasping point of the selected swing limb
        gait_planning_param = upd_swing_next_pos_max_stride_to_goal(gait_planning_param, SV, des_SV, LP, POS_e, time, surface_param);
        %%% 5) Plan the next position for the base
        gait_planning_param = upd_base_next_pos_crawl_gait(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, base_height, inc, time);
        %%% 6) Requirement check if the planned motion is kinematically feasible or not. If this is not satisfied, go back to 2)
        gait_planning_param = upd_kinematic_feasibility(SV, des_SV, LP, POS_e, gait_planning_param);
    end

    %%% 6) Record the footholds of swing leg
    gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);

end

case 'diagonal_gait_for_discrete_footholds'
    
if sum(des_SV.sup) == LP.num_limb    
	%%% 0) update the graspable points in all reachable region of limbs
    gait_planning_param = upd_graspable_points_in_reachable_area(SV,des_SV,LP,surface_param, gait_planning_param, sensing_camera_param);
    
    %%% 1) Select new swing leg
    gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time);
    
    %%% 2) Update Moving Direction (TO BE ADDED)
    
    %%% 3) Update walking sequence based on the moving direction
    gait_planning_param = upd_walking_sequence_based_on_moving_direction(gait_planning_param, des_SV, SV, LP, time);
      
    %%% Necessary temporary variable declaration @TODO: this should be in
    %%% the function and remove "sum(des_SV.sup) == LP.num_limb" from this
    %%% switch sentence brock.
    % kinematics feasibility chech flag
    gait_planning_param.kinematic_feasibility_check_is_OK = false;
    % Graspable points in Reachable area of each limb @TODO: probably we do
    % not have to prepare this temp variables for each limb by each. --> we
    % can make one single variable for all
    gait_planning_param.LF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,1);
    gait_planning_param.LH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,2);
    gait_planning_param.RH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,3);
    gait_planning_param.RF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,4);
        
    gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride = 0; % Index number used to identify the max value in the vector
    while ~gait_planning_param.kinematic_feasibility_check_is_OK
        %%% 4) Select the next grasping point of the selected swing limb
        gait_planning_param = upd_swing_next_pos_max_stride_to_goal(gait_planning_param, SV, des_SV, LP, POS_e, time, surface_param);
        %%% 5) Plan the next position for the base
        gait_planning_param = upd_base_next_pos_CoM_projection_on_intersection_of_diagonals(SV,des_SV,LP,gait_planning_param,POS_e,inc,surface_param,base_height,time);
        %%% 6) Requirement check if the planned motion is kinematically feasible or not. If this is not satisfied, go back to 2)
        gait_planning_param = upd_kinematic_feasibility(SV, des_SV, LP, POS_e, gait_planning_param);
    end

    %%% 6) Record the footholds of swing leg
    gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
end



case 'nonperiodic_gait_for_discrete_footholds'

if sum(des_SV.sup) == LP.num_limb % @TODO: this if sentence is written in each following function as well, so might be removed.
    %%% 0) update the graspable points in all reachable region of limbs
    gait_planning_param = upd_graspable_points_in_reachable_area(SV,des_SV,LP,surface_param, gait_planning_param, sensing_camera_param);

    %%% Necessary temporary variable declaration @TODO: this should be in
    %%% the function and remove "sum(des_SV.sup) == LP.num_limb" from this
    %%% switch sentence brock.
    gait_planning_param.kinematic_feasibility_check_is_OK = false;
    % Graspable points in Reachable area of each limb @TODO: probably we do
    % not have to prepare this temp variables for each limb by each. --> we
    % can make one single variable for all
    gait_planning_param.LF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,1);
    gait_planning_param.LH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,2);
    gait_planning_param.RH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,3);
    gait_planning_param.RF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,4);
    
    %%% 1) Update Moving Direction (TO BE ADDED)
    
    while ~gait_planning_param.kinematic_feasibility_check_is_OK
        %%% 2) Select new swing leg
        gait_planning_param = upd_swing_num_based_on_num_of_graspable_options(gait_planning_param, des_SV, SV, LP, POS_e, surface_param);
        %%% 3) Select the next grasping point of the selected swing limb
        gait_planning_param = upd_swing_next_pos_max_stride_to_goal(gait_planning_param, SV, des_SV, LP, POS_e, time, surface_param);
        %%% 4) Plan the next position for the base
        gait_planning_param = upd_base_next_pos_CoM_projection_on_intersection_of_diagonals(SV,des_SV,LP,gait_planning_param,POS_e,inc,surface_param,base_height,time);
        %%% 5) Requirement check if the planned motion is kinematically feasible or not. If this is not satisfied, go back to 2)
        gait_planning_param = upd_kinematic_feasibility(SV, des_SV, LP, POS_e, gait_planning_param);
    end
    
    %%% 6) Record the footholds of swing leg 
    gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
    
end
    
case 'GIA_opt_based_pose_planner' % this is under development and does not work in the current version.
    
if sum(des_SV.sup) == LP.num_limb && (round(rem(round(time - d_time,4),gait_planning_param.T_d),3) == 0 || time == 0)
    %%% 0) update the graspable points in all reachable region of limbs
    gait_planning_param = upd_graspable_points_in_reachable_area(SV,des_SV,LP,surface_param, gait_planning_param, sensing_camera_param);
    %%% Select next motion part 
    if ~gait_planning_param.base_move
        gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time);
        % move base to a safe position before swing leg motion 
        if gait_planning_param.swing_number == 3 || gait_planning_param.swing_number == 2
            gait_planning_param.base_move = true; % flag 
        end
    else
        gait_planning_param.base_move = false;
    end
    %%% Necessary temporary variable declaration @TODO: this should be in
    %%% the function and remove "sum(des_SV.sup) == LP.num_limb" from this
    %%% switch sentence brock.
    % kinematics feasibility check flag
%     gait_planning_param.kinematic_feasibility_check_is_OK = false;
    % Graspable points in Reachable area of each limb @TODO: probably we do
    % not have to prepare this temp variables for each limb by each. --> we
    % can make one single variable for all
    gait_planning_param.LF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,1);
    gait_planning_param.LH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,2);
    gait_planning_param.RH_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,3);
    gait_planning_param.RF_graspable_points_in_reachable_area = gait_planning_param.graspable_points_in_reachable_area(:,:,4);
        
    gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride = 0; % Index number used to identify the max value in the vector
    
    switch gait_planning_param.base_move
        % update the next desired position and orientation for base before swing leg motion if base_move is true  
        case true
            % Timing
            gait_planning_param.base_T = [time; time + gait_planning_param.T_d];
            %%% 3) Plan the next position for the base
            gait_planning_param = upd_base_next_pos_GIA_opt(gait_planning_param, des_SV, SV, LP, POS_e, time);
            gait_planning_param.POS_cur = POS_e;
            gait_planning_param.POS_next = POS_e;
        case false
            %%% 2) Select the next grasping point of the selected swing limb
            gait_planning_param.base_next = SV.R0;
            gait_planning_param.base_cur = SV.R0;
            gait_planning_param = upd_swing_next_pos_max_stride_to_goal(gait_planning_param, SV, des_SV, LP, POS_e, time, surface_param);
            % Timing
            gait_planning_param.leg_T = [time; time + gait_planning_param.T_d];
            gait_planning_param.base_T = [time; time + gait_planning_param.T_d];
            %%% 3) Record the footholds of swing leg
            gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time);
    end
    
end
otherwise
	disp('Invalid gait type');

end

end