%%%%%% Update
%%%%%% upd_swing_traject_spline3_two_segments_time
%%%%%% 
%%%%%% Update spline trajectory coefficients for swing leg
%%%%%% 
%%%%%% Created 2021-04-20
%%%%%% Warley Ribeiro
%
%
% Update coefficients for cubic spline of swing leg trajectory
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param : Parameters for motion planning (class)
%         gait_planning_param    : Parameters for Gait Planning (class)
%
%         motion_planning_param.spline_coeff_leg: Coefficients for spline curve (3xnx(m+1)x2 matrix))
%								1st dim: x-y-z coordinates (1 to 3)
%								2nd dim: leg number (1 to n)
%								3rd dim: coefficient index (1 to 4)
%								4th dim: segment number (1 or 2)
%     INPUT
%         gait_planning_param    : Parameters for Gait Planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%         time                   : Simulation time (scalar)
%		  LP                     : Link parameters (SpaceDyn class)

function [motion_planning_param, gait_planning_param] = upd_swing_traject_spline3_two_segments_time(gait_planning_param, motion_planning_param, time, LP)

global d_time;

% Define new trajectory for swing leg
% Check if limb is beginning to swing
for i = 1:LP.num_limb
	if abs(time - gait_planning_param.leg_T(1,i)) < d_time

		
		% Midpoint position
		gait_planning_param.POS_mid(1:2,i) = 0.5*(gait_planning_param.POS_cur(1:2,i) + gait_planning_param.POS_next(1:2,i));
		gait_planning_param.POS_mid(3,i) = gait_planning_param.POS_cur(3,i) + gait_planning_param.step_height;
		% Midpoint acceleration
		xm_dd = [ 0; 0; 0 ];
		t_m = 0.5*(gait_planning_param.leg_T(1,i) + gait_planning_param.leg_T(2,i));


		% x and y
		for j = 1:2
			% First segment
  	    	CC_1 = upd_spline3_pos_vel_acc(gait_planning_param.POS_cur(j,i), gait_planning_param.POS_mid(j,i),...
  	    	                             0, xm_dd(j), gait_planning_param.leg_T(1,i), t_m);
			% Second segment
        	CC_2 = upd_spline3_pos_acc_vel(gait_planning_param.POS_mid(j,i), gait_planning_param.POS_next(j,i),...
        	                             xm_dd(j), 0, t_m, gait_planning_param.leg_T(2,i));

        	for k = 1:size(CC_1)
	        	motion_planning_param.spline_coeff_leg(j,i,k,1) = CC_1(k); % xyz coord, leg_num, spline_coeff, segment_num
	        	motion_planning_param.spline_coeff_leg(j,i,k,2) = CC_2(k);
	        end
		end
		% z
		j = 3;
		% First segment
		CC_1 = upd_spline3_pos_vel(gait_planning_param.POS_cur(j,i), gait_planning_param.POS_mid(j,i),...
  	    	                        0, 0, gait_planning_param.leg_T(1,i), t_m);
		% Second segment
        CC_2 = upd_spline3_pos_vel(gait_planning_param.POS_mid(j,i), gait_planning_param.POS_next(j,i),...
        	                        0, 0, t_m, gait_planning_param.leg_T(2,i));
		for k = 1:size(CC_1)
	    	motion_planning_param.spline_coeff_leg(j,i,k,1) = CC_1(k); 
	        motion_planning_param.spline_coeff_leg(j,i,k,2) = CC_2(k);
	    end		
    end
end


end