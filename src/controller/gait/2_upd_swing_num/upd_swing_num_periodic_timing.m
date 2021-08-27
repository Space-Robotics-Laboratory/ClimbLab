%%%%%% Update
%%%%%% upd_swing_num_periodic_timing
%%%%%% 
%%%%%% Update swing legs number based on periodic timing 
%%%%%% 
%%%%%% Created: 2021-04-19
%%%%%% Warley Ribeiro
%
%
% Update number of swing leg for periodic gait, and may include several swing limbs at the same time
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for path planning (class)
%
%         gait_planning_param.swing_number      : Number of the selected leg to be the next swing leg (vector)
%
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_num_periodic_timing(gait_planning_param, LP, time)
    % initialize variable
    gait_planning_param.swing_number = [];
    for i = 1:LP.num_limb
        % Swing limb according to each limb swing timing
        if time >= gait_planning_param.leg_T(1,i) && time < gait_planning_param.leg_T(2,i) 
            if isempty(gait_planning_param.swing_number) 
                gait_planning_param.swing_number = i;
            % Include more swing limbs 
            else
                gait_planning_param.swing_number(end+1) = i;
            end
        end
    end

end