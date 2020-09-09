%%%%%% Update
%%%%%% upd_swing_traject_spline3_two_segments
%%%%%% 
%%%%%% Update spline trajectory coefficients for swing leg
%%%%%% 
%%%%%% Created 2020-04-14
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-14
%
%
% Update coefficients for cubic spline of swing leg trajectory
%
% Function variables:
%
%     OUTPUT
%         motion_planning_param : Parameters for motion planning (class)
%         path_planning_param   : Parameters for path planning (class)
%
%         motion_planning_param.spline_coeff_leg: Coefficients for spline curve (3xnx(m+1)x2 matrix))
%								1st dim: x-y-z coordinates (1 to 3)
%								2nd dim: leg number (1 to n)
%								3rd dim: coefficient index (1 to 4)
%								4th dim: segment number (1 or 2)
%     INPUT
%         gait_param             : Parameters for gait (class)
%         path_planning_param    : Parameters for path planning (class)
%         motion_planning_param  : Parameters for motion planning (class)
%		  LP                     : Link parameters (SpaceDyn class)
%         des_SV      			 : Desired state variables (SpaceDyn class)

function [motion_planning_param, path_planning_param] = upd_swing_traject_spline3_two_segments(path_planning_param, gait_param, motion_planning_param, des_SV, LP)

% Define new trajectory for swing leg
	% If all legs are in support phase
	if sum(des_SV.sup) == LP.num_limb

		i = path_planning_param.swing_number;
		
		% Midpoint position
		path_planning_param.POS_mid(1:2,i) = 0.5*(path_planning_param.POS_cur(1:2,i) + path_planning_param.POS_next(1:2,i));
		path_planning_param.POS_mid(3,i) = path_planning_param.POS_cur(3,i) + gait_param.step_height;
		% Midpoint acceleration
		xm_dd = [ 0; 0; 0 ];
		t_m = 0.5*(path_planning_param.leg_T(1) + path_planning_param.leg_T(2));


		% x and y
		for j = 1:2
			% First segment
  	    	CC_1 = upd_spline3_pos_vel_acc(path_planning_param.POS_cur(j,i), path_planning_param.POS_mid(j,i),...
  	    	                             0, xm_dd(j), path_planning_param.leg_T(1), t_m);
			% Second segment
        	CC_2 = upd_spline3_pos_acc_vel(path_planning_param.POS_mid(j,i), path_planning_param.POS_next(j,i),...
        	                             xm_dd(j), 0, t_m, path_planning_param.leg_T(2));

        	for k = 1:size(CC_1)
	        	motion_planning_param.spline_coeff_leg(j,i,k,1) = CC_1(k); % xyz coord, leg_num, spline_coeff, segment_num
	        	motion_planning_param.spline_coeff_leg(j,i,k,2) = CC_2(k);
	        end
		end
		% z
		j = 3;
		% First segment
		CC_1 = upd_spline3_pos_vel(path_planning_param.POS_cur(j,i), path_planning_param.POS_mid(j,i),...
  	    	                        0, 0, path_planning_param.leg_T(1), t_m);
		% Second segment
        CC_2 = upd_spline3_pos_vel(path_planning_param.POS_mid(j,i), path_planning_param.POS_next(j,i),...
        	                        0, 0, t_m, path_planning_param.leg_T(2));
		for k = 1:size(CC_1)
	    	motion_planning_param.spline_coeff_leg(j,i,k,1) = CC_1(k); 
	        motion_planning_param.spline_coeff_leg(j,i,k,2) = CC_2(k);
	    end		
	end


end