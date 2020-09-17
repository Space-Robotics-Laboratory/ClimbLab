%%%%%% Update
%%%%%% upd_base_next_crawl_fixed_stride
%%%%%% 
%%%%%% Update next base position for crawl gait
%%%%%% 
%%%%%% Created 2020-04-13
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-15
%
%
% Update next base position for crawl gait with fixed stride, considering that the base moves through the entire cycle
%
% Function variables:
%
%     OUTPUT
%         path_planning_param   : Parameters for path planning (class)
%
%         path_planning_param.base_cur      : Current position of the base when selecting a new one [m] (3x1 vector)
%         path_planning_param.base_next     : Next desired position of the base [m] (3x1 vector)
%         path_planning_param.base_T        : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         path_planning_param   : Parameters for path planning (class)
%         gait_param            : Parameters for gait (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         SV                    : State values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function path_planning_param = upd_base_next_crawl_fixed_stride(path_planning_param, gait_param, des_SV, SV, LP, time)

	%%% Select next position for the base
	% If all legs are in support phase
	if sum(des_SV.sup) == LP.num_limb
    % Begin of a new cycle
	    if path_planning_param.swing_number == gait_param.sequence(1)
	        % Current position of the base
	        path_planning_param.base_cur = SV.R0;
	        % Next position of the base
	        path_planning_param.base_next = path_planning_param.base_cur + [gait_param.step_length; 0; 0];    
	        % Timing
	        path_planning_param.base_T = [time; time + gait_param.T];    
	    end
	end 

end