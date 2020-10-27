%%%%%% Update
%%%%%% upd_path_planning
%%%%%% 
%%%%%% Update path planning variables
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-11 by Kentaro Uno
%
%
% Update the path planning based on the gait type
%
% Function variables:
%
%     OUTPUT
%         path_planning_param   : Parameters for path planning (class)
%     INPUT
%         path_planning_param   : Parameters for path planning (class)
%         gait_param            : Parameters for gait (class)
%         surface_param         : Parameters for surface (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         SV                    : State values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         environment_param     : Parameters for environment (class)
%         robot_param           : Parameters for robot (class)
%         time                  : Simulation time [s] (scalar)

function path_planning_param = upd_path_planning(path_planning_param, gait_param, surface_param, des_SV, SV, LP, POS_e, environment_param, robot_param, time)

inc = environment_param.inc;
base_height = robot_param.base_height;
switch gait_param.type

case 'crawl_fixed_stride'
	% Select new swing leg
	path_planning_param = upd_swing_number_crawl_fixed_stride(path_planning_param, gait_param, des_SV, LP, time);
	% Select next grasping point
	path_planning_param = upd_swing_next_pos_crawl_fixed_stride(path_planning_param, gait_param, des_SV, LP, POS_e, time);
	% Select next position for the base
	path_planning_param = upd_base_next_crawl_fixed_stride(path_planning_param, gait_param, des_SV, SV, LP, time);

case 'do_nothing'
	path_planning_param = [];
        
case 'crawl_uno_ver'

    % update the graspable points in all reachable region of limbs
    path_planning_param = upd_graspable_points_in_reachable_area(SV,LP,surface_param,path_planning_param);
    % Select new swing leg 
    path_planning_param = upd_swing_number_crawl_fixed_stride(path_planning_param, gait_param, des_SV, LP, time);
	% Select next grasping point of the selected swing limb
    path_planning_param = upd_swing_next_pos_crawl_uno_ver(path_planning_param, gait_param, surface_param, SV, des_SV, LP, POS_e, inc, base_height, time);

case 'nonperiodic_uno_ver'
	
    % update the graspable points in all reachable region of limbs
    path_planning_param = upd_graspable_points_in_reachable_area(SV,LP,surface_param,path_planning_param);
    % Select new swing leg and its next grasping point of the selected swing limb
	path_planning_param = upd_swing_num_and_grasping_point_based_on_graspable_options(path_planning_param, gait_param, surface_param, des_SV, SV, LP, POS_e, inc, base_height, time);
    
    otherwise
	disp('Invalid gait type');
end

end