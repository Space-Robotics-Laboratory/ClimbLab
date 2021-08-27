%%%%%% Update
%%%%%% upd_swing_timing
%%%%%% 
%%%%%% Update swing legs timing
%%%%%% 
%%%%%% Created: 2021-04-19
%%%%%% Warley Ribeiro
%
%
% Update the timing for the swing phase 
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for path planning (class)
%
%         gait_planning_param.leg_T      : initial and final swing time of each leg (2xn matrix)
%
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_timing(gait_planning_param, LP, time)

    global d_time;
    
	% Beginning of new cycle
    if rem(time, gait_planning_param.T) < d_time
        for i = 1:LP.num_limb
            % Each leg initial and final swing time 
            gait_planning_param.leg_T(1,i) = time + gait_planning_param.phi(i);
            gait_planning_param.leg_T(2,i) = gait_planning_param.leg_T(1,i) + gait_planning_param.T_d;
        end
    end

end