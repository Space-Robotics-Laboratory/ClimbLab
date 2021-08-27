%%%%%% Update
%%%%%% upd_swing_current_des_POS_spline3_two_segments_for_intermittent_gait
%%%%%% 
%%%%%% Calculate current desired position for the swing limb from the cubic
%%%%%% spline trajectory, For a gait that moves legs and base separately 
%%%%%% 
%%%%%% Created 2020-04-14
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-12-21
%%%%%% Yusuke Koizumi
%
%
% Calculate current position for the swing limb considering a cubic spline curve connected by a mid-point after half of the
% swing period
%
%                              x(t) = a0 + a1*t + a2*t^2 + a3*t^3
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param : Parameters for motion planning (class)
%         gait_planning_param   : Parameters for Gait Planning (class)
%
%         motion_planning_param.spline_coeff_base: Coefficients for spline curve (3x1x(m+1)x2 matrix))
%								1st dim: x-y-z coordinates (1 to 3)
%								2nd dim: leg number (1 to n)
%								3rd dim: coefficient index (1 to 4)
%								4th dim: segment number (1 or 2)
%     INPUT
%         gait_planning_param    : Parameters for Gait Planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%         LP                     : Link parameters (SpaceDyn class)
%         cont_POS               : Contact position [m] (3xn matrix)
%         time                   : Simulation time [s] (scalar)

function motion_planning_param = upd_swing_current_des_POS_spline_for_intermittent_gait(gait_planning_param, motion_planning_param, LP, cont_POS, time)
if ~gait_planning_param.base_move
    for i = 1:LP.num_limb
        % Current swing limb
        if i == gait_planning_param.swing_number
            % Check if before or after mid-point
            if time <= 0.5*(gait_planning_param.leg_T(1) + gait_planning_param.leg_T(2))
                j = 1;
            else
                j = 2;
            end
            % Desired position for swing leg
            motion_planning_param.des_POS_e(:,i) = motion_planning_param.spline_coeff_leg(:,i,1,j) + ...
                motion_planning_param.spline_coeff_leg(:,i,2,j)*time + ...
                motion_planning_param.spline_coeff_leg(:,i,3,j)*time.^2 + ...
                motion_planning_param.spline_coeff_leg(:,i,4,j)*time.^3;
        else
            % Desired position for support leg
            motion_planning_param.des_POS_e(:,i) = cont_POS(:,i);
        end
    end
else
    motion_planning_param.des_POS_e = cont_POS;
end

end