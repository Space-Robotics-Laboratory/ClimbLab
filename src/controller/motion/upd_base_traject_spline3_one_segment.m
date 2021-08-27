%%%%%% Update
%%%%%% upd_base_traject_spline3_one_segment
%%%%%% 
%%%%%% Update spline trajectory coefficients for swing leg
%%%%%% 
%%%%%% Created 2020-04-14
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-05-04
%%%%%% Keigo Haji
%
%
% Update coefficients for cubic spline of base trajectory
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param  : Parameters for motion planning (class)
%         gait_planning_param    : Parameters for Gait Planning (class)
%
%         motion_planning_param.spline_coeff_base: Coefficients for spline curve (3x1x(m+1)x2 matrix))
%								1st dim: x-y-z coordinates (1 to 3)
%								2nd dim: leg number (1 to n)
%								3rd dim: coefficient index (1 to 4)
%     INPUT
%         gait_planning_param    : Parameters for Gait Planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%         des_SV      			 : Desired State variable (SpaceDyn class)
%         LP      				 : Link Parameters (SpaceDyn class)

function [motion_planning_param, gait_planning_param] = upd_base_traject_spline3_one_segment(gait_planning_param, motion_planning_param, des_SV, LP)

% Define new trajectory for base
	% If all legs are in support phase
    if sum(des_SV.sup) == LP.num_limb
        % Begin of a new cycle @TODO: instead of using the gait name
        % switch, use gait_planning_param to change move base at each step
        % or only at the beginning of each cycle
        if (contains(gait_planning_param.type, 'crawl') && gait_planning_param.swing_number == gait_planning_param.sequence(1)) ...
                || (contains(gait_planning_param.type, 'crawl') && gait_planning_param.swing_number == gait_planning_param.sequence(3)) ...
                || (contains(gait_planning_param.type, 'crawl') && gait_planning_param.crawl_gait_sequence_change_flag == true) ...
                || contains(gait_planning_param.type,'diagonal_gait') ...
                || strcmp(gait_planning_param.type,'nonperiodic_gait_for_discrete_footholds') ...
                || strcmp(gait_planning_param.type,'GIA_opt_based_pose_planner')
            % x, y and z
            for j = 1:3
                % Single segment
                CC_1 = upd_spline3_pos_vel(gait_planning_param.base_cur(j), gait_planning_param.base_next(j),...
                    0, 0, gait_planning_param.base_T(1), gait_planning_param.base_T(2));
                for k = 1:size(CC_1)
                    motion_planning_param.spline_coeff_base(j,1,k) = CC_1(k); % xyz coord, leg_num, spline_coeff, segment_num
                end
            end
        end
    end


end