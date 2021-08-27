%%%%%% Update
%%%%%% upd_base_next_pos_periodic_fixed_stride
%%%%%% 
%%%%%% Update next base position for periodic gait
%%%%%% 
%%%%%% Created: 2021-04-20
%%%%%% Warley Ribeiro
%
%
% Update next base position for periodic gait with fixed stride, considering that the base moves through the entire cycle
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%
%         gait_planning_param.base_cur      : Current position of the base when selecting a new one [m] (3x1 vector)
%         gait_planning_param.base_next     : Next desired position of the base [m] (3x1 vector)
%         gait_planning_param.base_T        : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         SV                    : State values (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_base_next_pos_periodic_fixed_stride(gait_planning_param, SV, time)

	global d_time;
    
	%%% Select next position for the base
	% Beginning of new cycle
    if rem(time, gait_planning_param.T) < d_time
		% Current position of the base
        gait_planning_param.base_cur = SV.R0;
	    % Next position of the base
	    gait_planning_param.base_next = gait_planning_param.base_cur + [gait_planning_param.step_length; 0; 0];    
	    % Timing
	    gait_planning_param.base_T = [time; time + gait_planning_param.T];    
    end 

end