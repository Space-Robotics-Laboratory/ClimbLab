%%%%%% Update
%%%%%% upd_swing_next_pos_max_stride_to_goal
%%%%%%
%%%%%% Update next swing limb's grasping position in such a way to obtain 
%%%%%% the longest stride from the graspable point options. 
%%%%%%
%%%%%% Created: 2020-06-12
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-07-06
%%%%%% Keigo Haji
%
%
% You can see the details of the planning method is shown in the following paper:
% --------------------------------------------------------------------
% In: Proceedings of the IEEE/SICE International Symposium on System 
% Integration (SII) 2019 by K. Uno et al.
% Proceedings Paper URL: 
% https://ieeexplore.ieee.org/document/8700455
% --------------------------------------------------------------------
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%
%         gait_planning_param.POS_cur       : Current position of the swing leg when selecting a new one [m] (3x1 vector)
%         gait_planning_param.POS_next      : Next desired position of the swing leg [m] (3x1 vector)
%         gait_planning_param.leg_T         : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%         gait_planning_param.base_T        : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         SV                    : State values (SpaceDyn class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         time                  : Simulation time [s] (scalar)


function gait_planning_param = upd_swing_next_pos_max_stride_to_goal(gait_planning_param, SV, des_SV, LP, POS_e, time, surface_param)
if sum(des_SV.sup) == LP.num_limb
    i = gait_planning_param.swing_number;

    % Current position of swing leg on the surface
    [gait_planning_param.POS_cur(1,i),gait_planning_param.POS_cur(2,i),...
        gait_planning_param.POS_cur(3,i)] = get_map_pos_of_graspable_points(POS_e(1,i),POS_e(2,i),surface_param);

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% dot product calculation for each GP_in_RA
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    switch i
        case 1
            GP_in_RA = gait_planning_param.LF_graspable_points_in_reachable_area;
        case 2
            GP_in_RA = gait_planning_param.LH_graspable_points_in_reachable_area;
        case 3
            GP_in_RA = gait_planning_param.RH_graspable_points_in_reachable_area;
        case 4
            GP_in_RA = gait_planning_param.RF_graspable_points_in_reachable_area;
    end
    
    % Set the Vector_Base_to_Goal)
    vec_bg = gait_planning_param.goal(:,gait_planning_param.goal_num) - SV.R0;
        
    % dot product calculation for each GP_in_RA
    dot_tmp = zeros(1,size(GP_in_RA,2));
    gait_planning_param.POS_next = POS_e;
    for ind = 1:size(GP_in_RA,2)
        vec_tmp = GP_in_RA(1:2,ind) - POS_e(1:2,i);
        % if the stride is too large, it is eliminated
        if norm(vec_tmp) <= gait_planning_param.allowable_max_stride
            dot_tmp(1,ind) = dot(vec_bg(1:2,1), vec_tmp);
        else
            dot_tmp(1,ind) = nan;
        end
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Select new grasping point
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [~,gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride] = max(dot_tmp);

    %%% new grasping point is selected.
    gait_planning_param.POS_next(:, i) = GP_in_RA(:,gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride);
            
    %%% Only if the all options does not satisfy the kinematic feasibility, the
    %%% current same position is selected, which should pass the check.
    if min( isnan(dot_tmp) ) == 1 % if all elements are nan, the input vector's minimum value is 1
        disp("The number of limb that has no feasible next graspable points: ");
        disp( i );
        error("in upd_swing_next_pos_max_stride_to_goal(): The best next gripping point does not exist. The solver does not update the next grasping point and base position.");
        gait_planning_param.POS_next(:,i) = POS_e(:,i);
%         gait_planning_param.base_next = SV.R0;
    end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Time for current and desired positions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Timing
    gait_planning_param.leg_T = [time; time + gait_planning_param.T_d];
    

end
    
end