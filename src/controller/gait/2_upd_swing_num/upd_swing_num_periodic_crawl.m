%%%%%% Update
%%%%%% upd_swing_num_periodic_crawl
%%%%%% 
%%%%%% Update number of swing leg for the typical periodic crawl gait
%%%%%% 
%%%%%% Created: 2020-04-13
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2020-11-18
%%%%%% Kentaro Uno
%
%
% Update number of swing leg for crawl gait with fixed stride
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for path planning (class)
%
%         gait_planning_param.swing_number      : Number of the selected leg to be the next swing leg (scalar)
%
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_num_periodic_crawl(gait_planning_param, des_SV, LP, time)

	%%% Select new swing leg
	% If all legs are in support phase
	if sum(des_SV.sup) == LP.num_limb
		% First step
		if time == 0
			gait_planning_param.swing_number = gait_planning_param.sequence(1);
		else
			sw_num = gait_planning_param.swing_number(1);
			seq = gait_planning_param.sequence;
			% Find previous swing leg
			previous_index = find(seq==sw_num);
			if previous_index == size(gait_planning_param.sequence,1)
				gait_planning_param.swing_number = gait_planning_param.sequence(1);
			else
				gait_planning_param.swing_number = gait_planning_param.sequence(previous_index+1);
			end
		end
	end

end