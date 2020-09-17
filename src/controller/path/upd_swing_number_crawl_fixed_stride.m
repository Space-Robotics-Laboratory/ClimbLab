%%%%%% Update
%%%%%% upd_swing_number_crawl_fixed_stride
%%%%%% 
%%%%%% Update number of swing leg for crawl gait
%%%%%% 
%%%%%% Created 2020-04-13
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-13
%
%
% Update number of swing leg for crawl gait with fixed stride
%
% Function variables:
%
%     OUTPUT
%         path_planning_param   : Parameters for path planning (class)
%
%         path_planning_param.swing_number      : Number of the selected leg to be the next swing leg (scalar)
%     INPUT
%         path_planning_param   : Parameters for path planning (class)
%         gait_param            : Parameters for gait (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function path_planning_param = upd_swing_number_crawl_fixed_stride(path_planning_param, gait_param, des_SV, LP, time)

	%%% Select new swing leg
	% If all legs are in support phase
	if sum(des_SV.sup) == LP.num_limb
		% First step
		if time == 0
			path_planning_param.swing_number = gait_param.sequence(1);
		else
			sw_num = path_planning_param.swing_number(1);
			seq = gait_param.sequence;
			% Find previous swing leg
			previous_index = find(seq==sw_num);
			if previous_index == size(gait_param.sequence,1)
				path_planning_param.swing_number = gait_param.sequence(1);
			else
				path_planning_param.swing_number = gait_param.sequence(previous_index+1);
			end
		end
	end

end