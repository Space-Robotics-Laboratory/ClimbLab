%%%%%% Update
%%%%%% upd_swing_next_pos_periodic_fixed_stride
%%%%%% 
%%%%%% Update next grasping position for crawl gait
%%%%%% 
%%%%%% Created: 2021-04-20
%%%%%% Warley Ribeiro
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
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_next_pos_periodic_fixed_stride(gait_planning_param, LP, POS_e, time)

	global d_time;
    
    % Initialize current position and next position
    if time == 0
        for j=1:LP.num_limb 
            % Current position of swing leg on the surface
            [gait_planning_param.POS_cur(1,j),gait_planning_param.POS_cur(2,j),...
                                             gait_planning_param.POS_cur(3,j)] = get_map_pos(POS_e(1,j),POS_e(2,j));
        end
        gait_planning_param.POS_next = gait_planning_param.POS_cur;
    end
    
    % Check if limb is beginning to swing
    for i = 1:LP.num_limb
        if abs(time - gait_planning_param.leg_T(1,i)) < d_time/2
            % Update current position
            gait_planning_param.POS_cur(:,i) = gait_planning_param.POS_next(:,i);
            % Next desired position
            gait_planning_param.POS_next(:,i) = gait_planning_param.POS_cur(:,i) + [gait_planning_param.step_length; 0; 0];

            % Next desired position on the surface
            [gait_planning_param.POS_next(1,i),gait_planning_param.POS_next(2,i),gait_planning_param.POS_next(3,i)] = ...
    	                                 get_map_pos(gait_planning_param.POS_next(1,i),gait_planning_param.POS_next(2,i));
        end
    end
                                     
	% Record the footholds of leg i
    if time == 0
        for j=1:LP.num_limb
        	gait_planning_param.footholds_history_limb(:,gait_planning_param.footholds_count_limb(j),j) = POS_e(:,j);
            gait_planning_param.footholds_count_limb(j) = gait_planning_param.footholds_count_limb(j) + 1;
        end
    end
    for i = 1:LP.num_limb
        if abs(time - gait_planning_param.leg_T(1,i)) < d_time
            gait_planning_param.footholds_history_limb(:,gait_planning_param.footholds_count_limb(i),i) = gait_planning_param.POS_next(:,i);
            gait_planning_param.footholds_count_limb(i) = gait_planning_param.footholds_count_limb(i) + 1;
        end
    end



end