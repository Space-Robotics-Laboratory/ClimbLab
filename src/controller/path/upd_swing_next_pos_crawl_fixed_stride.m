%%%%%% Update
%%%%%% upd_swing_next_pos_crawl_fixed_stride
%%%%%% 
%%%%%% Update next grasping position for crawl gait
%%%%%% 
%%%%%% Created 2020-04-13
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-13
%
%
% Update next grasping position for crawl gait with fixed stride, based on the map
%
% Function variables:
%
%     OUTPUT
%         path_planning_param   : Parameters for path planning (class)
%
%         path_planning_param.POS_cur       : Current position of the swing leg when selecting a new one [m] (3x1 vector)
%         path_planning_param.POS_next      : Next desired position of the swing leg [m] (3x1 vector)
%         path_planning_param.leg_T         : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         path_planning_param   : Parameters for path planning (class)
%         gait_param            : Parameters for gait (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         time                  : Simulation time [s] (scalar)

function path_planning_param = upd_swing_next_pos_crawl_fixed_stride(path_planning_param, gait_param, des_SV, LP, POS_e, time)

	%%% Select next grasping point
	% If all legs are in support phase
	if sum(des_SV.sup) == LP.num_limb
		% Current swing leg
		i = path_planning_param.swing_number;

		% Current position of swing leg on the surface
    	[path_planning_param.POS_cur(1,i),path_planning_param.POS_cur(2,i),...
    	                                 path_planning_param.POS_cur(3,i)] = get_map_pos(POS_e(1,i),POS_e(2,i));
	    % Next desired position
    	path_planning_param.POS_next(:,i) = path_planning_param.POS_cur(:,i) + [gait_param.step_length; 0; 0];

    	% Next desired position on the surface
    	[path_planning_param.POS_next(1,i),path_planning_param.POS_next(2,i),path_planning_param.POS_next(3,i)] = ...
    	                                 get_map_pos(path_planning_param.POS_next(1,i),path_planning_param.POS_next(2,i));
                                     
        % Record the footholds of leg i
        if time == 0
            for j=1:LP.num_limb
                path_planning_param.footholds_history_limb(:,path_planning_param.footholds_count_limb(j),j) = POS_e(:,j);
                path_planning_param.footholds_count_limb(j) = path_planning_param.footholds_count_limb(j) + 1;
            end
        end
        path_planning_param.footholds_history_limb(:,path_planning_param.footholds_count_limb(i),i) = path_planning_param.POS_next(:,i);
        path_planning_param.footholds_count_limb(i) = path_planning_param.footholds_count_limb(i) + 1;
		
        % Time for current and desired positions
		path_planning_param.leg_T = [time; time + gait_param.T_d];
	end


end