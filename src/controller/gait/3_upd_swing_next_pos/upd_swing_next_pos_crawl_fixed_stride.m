%%%%%% Update
%%%%%% upd_swing_next_pos_crawl_fixed_stride
%%%%%% 
%%%%%% Update next grasping position for crawl gait
%%%%%% 
%%%%%% Created: 2020-04-13
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2021-04-29
%%%%%% Keigo Haji
%
%
% Update next grasping position for crawl gait with fixed stride, based on the map
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%
%         gait_planning_param.POS_cur       : Current position of the swing leg when selecting a new one [m] (3x1 vector)
%         gait_planning_param.POS_next      : Next desired position of the swing leg [m] (3x1 vector)
%         gait_planning_param.leg_T         : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         SV                    : State values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_next_pos_crawl_fixed_stride(gait_planning_param, des_SV, SV, LP, POS_e, time)

%%% Select next grasping point
% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb
    % Current swing leg
    i = gait_planning_param.swing_number;
    
    for j=1:LP.num_limb
        % Current position of swing leg on the surface
        [gait_planning_param.POS_cur(1,j),gait_planning_param.POS_cur(2,j),...
            gait_planning_param.POS_cur(3,j)] = get_map_pos(POS_e(1,j),POS_e(2,j));
    end
    
    % Next desired position
    gait_planning_param.POS_next = POS_e; % initialize for all limb
    
    % Set the direction of stride to goal
    vec_base_to_goal = gait_planning_param.goal(1:2,gait_planning_param.goal_num) -  SV.R0(1:2);
    unit_vec_base_to_goal = vec_base_to_goal/ norm(vec_base_to_goal);
    fix_stride_to_goal = unit_vec_base_to_goal * gait_planning_param.step_length;
    
    gait_planning_param.POS_next(:,i) = gait_planning_param.POS_cur(:,i) + [fix_stride_to_goal(1); fix_stride_to_goal(2); 0];
    % Next desired position on the surface
    [gait_planning_param.POS_next(1,i),gait_planning_param.POS_next(2,i),gait_planning_param.POS_next(3,i)] = ...
        get_map_pos(gait_planning_param.POS_next(1,i),gait_planning_param.POS_next(2,i));
    
    % Time for current and desired positions
    gait_planning_param.leg_T = [time; time + gait_planning_param.T_d];
end


end