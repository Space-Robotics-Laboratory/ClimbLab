%%%%%% Update
%%%%%% upd_base_current_des_POS_spline3_one_segment
%%%%%% 
%%%%%% Calculate current desired position for base from cubic spline trajectory
%%%%%% 
%%%%%% Created 2020-04-14
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-16
%
%
% Calculate current position for base considering a cubic spline curve
%
%                              x(t) = a0 + a1*t + a2*t^2 + a3*t^3
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param  : Parameters for motion planning (class)
%     INPUT
%         path_planning_param    : Parameters for path planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%         LP      				 : Link Parameters (SpaceDyn class)
%         time     				 : Simulation time [s] (scalar)

function motion_planning_param = upd_base_current_des_POS_spline3_one_segment(path_planning_param, motion_planning_param, LP, time)

% Desired position for base
motion_planning_param.des_R0(:) = motion_planning_param.spline_coeff_base(:,1,1) + ...
                                  motion_planning_param.spline_coeff_base(:,1,2)*time + ...
                                  motion_planning_param.spline_coeff_base(:,1,3)*time.^2 + ...
                                  motion_planning_param.spline_coeff_base(:,1,4)*time.^3;
    

end