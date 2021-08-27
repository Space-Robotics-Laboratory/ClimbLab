%%%%%% Update
%%%%%% upd_base_next_pos_crawl_gait
%%%%%%
%%%%%% Update next base position for crawl gait
%%%%%%
%%%%%% Created: 2021-04-29
%%%%%% Keigo Haji
%%%%%% Last updated: 2021-05-12
%%%%%% Kentaro Uno
%
%
% Update next base position for crawl gait, considering that the
% intersection of a diagonal line and a moving direction vector
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
%         gait_planning_param   : Parameters for path planning (class)
%         surface_param         : Parameters for surface (class)
%         SV                    : State values (SpaceDyn class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         inc                   : Surface inclination [deg] (scalar)
%         base_height           : Height of base relative to map [m]
%         time                  : Simulation time [s] (scalar)



function gait_planning_param = upd_base_next_pos_crawl_gait(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, base_height, inc, time)

%%% Select next position for the base

% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb
    
    % When the sequence is not changed, robot keeps walking and the
    % nexe base position is set on a intersection of a diagonal line
    % and a moving direction vector
    % Plan before starting the hind leg swing period
    if gait_planning_param.crawl_gait_sequence_change_flag == false ...
            && (gait_planning_param.swing_number == gait_planning_param.sequence(1) || gait_planning_param.swing_number == gait_planning_param.sequence(3))
        
        % Save current position of the base: this is w.r.t. the frame fixed
        % on the inclined plane
        gait_planning_param.base_cur = SV.R0;
        
        % Set the Vector_Base_to_Goal
        vec_bg = gait_planning_param.goal(:,gait_planning_param.goal_num) - SV.R0;
             
        %%% Set the next base based on a intersection of a diagonal line and a moving direction vector
        %%% NOTE THAT: This is done in the inertial frame which z axis is parallel to
        %%% the gravity vector
        gait_planning_param.POS_next_wrt_inertial_frame = (rpy2dc([0;pi*inc/180;0])) * gait_planning_param.POS_next;
        POS_e_wrt_inertial_frame = (rpy2dc([0;pi*inc/180;0])) * POS_e;
        gait_planning_param.base_cur_wrt_inertial_frame = (rpy2dc([0;pi*inc/180;0])) * gait_planning_param.base_cur;
        vec_bg_wrt_inertial_frame = (rpy2dc([0;pi*inc/180;0])) * vec_bg;
        
        
        % prepare some temporary variables in advance
        L1_x = zeros(2,1); % x positions of first diagonal line (connecting limb 1 and limb 3 supporting points)
        L1_y = zeros(2,1); % y positions of first diagonal line (connecting limb 1 and limb 3 supporting points)
        L2_x = zeros(2,1); % x positions of second diagonal line (connecting limb 2 and limb 4 supporting points)
        L2_y = zeros(2,1); % y positions of second diagonal line (connecting limb 2 and limb 4 supporting points)
        L3_x = zeros(2,1); % x positions of line connecting currenct base pos and moving direction
        L3_y = zeros(2,1); % y positions of line connecting currenct base pos and moving direction
        
        % Insert the positions of next POS_e
        for j = 1:LP.num_limb
            if j == gait_planning_param.swing_number
                switch j
                    case 1
                        L1_x(1) = gait_planning_param.POS_next_wrt_inertial_frame(1,j);
                        L1_y(1) = gait_planning_param.POS_next_wrt_inertial_frame(2,j);
                    case 2
                        L2_x(1) = gait_planning_param.POS_next_wrt_inertial_frame(1,j);
                        L2_y(1) = gait_planning_param.POS_next_wrt_inertial_frame(2,j);
                    case 3
                        L1_x(2) = gait_planning_param.POS_next_wrt_inertial_frame(1,j);
                        L1_y(2) = gait_planning_param.POS_next_wrt_inertial_frame(2,j);
                    case 4
                        L2_x(2) = gait_planning_param.POS_next_wrt_inertial_frame(1,j);
                        L2_y(2) = gait_planning_param.POS_next_wrt_inertial_frame(2,j);
                end
            else
                switch j
                    case 1
                        L1_x(1) = POS_e_wrt_inertial_frame(1,j);
                        L1_y(1) = POS_e_wrt_inertial_frame(2,j);
                    case 2
                        L2_x(1) = POS_e_wrt_inertial_frame(1,j);
                        L2_y(1) = POS_e_wrt_inertial_frame(2,j);
                    case 3
                        L1_x(2) = POS_e_wrt_inertial_frame(1,j);
                        L1_y(2) = POS_e_wrt_inertial_frame(2,j);
                    case 4
                        L2_x(2) = POS_e_wrt_inertial_frame(1,j);
                        L2_y(2) = POS_e_wrt_inertial_frame(2,j);
                end
            end
        end
        
        % Insert the positions of current base positons and
        % vector_base_to_goal in L3
        L3_x = [gait_planning_param.base_cur_wrt_inertial_frame(1); gait_planning_param.base_cur_wrt_inertial_frame(1) + vec_bg_wrt_inertial_frame(1)];
        L3_y = [gait_planning_param.base_cur_wrt_inertial_frame(2); gait_planning_param.base_cur_wrt_inertial_frame(2) + vec_bg_wrt_inertial_frame(2)];
        
        if gait_planning_param.swing_number == 2 || gait_planning_param.swing_number == 4
            % Calculate the intersection of vec_bg and the diagonal line
            % connecting limb 2&4
            Dx12 = L3_x(2)-L3_x(1);
            Dx34 = L2_x(2)-L2_x(1);
            Dy12 = L3_y(2)-L3_y(1);
            Dy34 = L2_y(2)-L2_y(1);
            Dx24 = L3_x(1)-L2_x(1);
            Dy24 = L3_y(1)-L2_y(1);
        elseif gait_planning_param.swing_number == 1 || gait_planning_param.swing_number == 3
            % Calculate the intersection of vec_bg and the diagonal line
            % connecting limb 1&3
            Dx12 = L3_x(2)-L3_x(1);
            Dx34 = L1_x(2)-L1_x(1);
            Dy12 = L3_y(2)-L3_y(1);
            Dy34 = L1_y(2)-L1_y(1);
            Dx24 = L3_x(1)-L1_x(1);
            Dy24 = L3_y(1)-L1_y(1);
        end
        
        % When the diagonal line is almost parallel to vec_bg, the
        % determinant of the following matrix cannot be calculated.
        % Thus, the robot base will not move because the intersection
        % cannot be found.
        if norm(det([Dx12 -Dx34; Dy12 -Dy34])) < 0.00001
            gait_planning_param.base_next =  gait_planning_param.base_cur;
        else
            % Solve the equation
            ts = [Dx12 -Dx34; Dy12 -Dy34] \ [-Dx24; -Dy24];
            % Take weighted combinations of points on the line of vec_bg
            gait_planning_param.base_next_wrt_inertial_frame = ts(1)*[L3_x(2); L3_y(2)] + (1-ts(1))*[L3_x(1); L3_y(1)];
            
            gait_planning_param.desired_displacement_wrt_inertial_frame = ...
                gait_planning_param.base_next_wrt_inertial_frame - gait_planning_param.base_cur_wrt_inertial_frame(1:2);
            %%% Set the next desired base position w.r.t. the frame used in
            %%% the ClimbLab back, that is fixed onto the inclined plane
            gait_planning_param.base_next = [ gait_planning_param.base_cur(1) + gait_planning_param.desired_displacement_wrt_inertial_frame(1)/cos(pi*inc/180); 
                                              gait_planning_param.base_cur(2) + gait_planning_param.desired_displacement_wrt_inertial_frame(2); 
                                              base_height + surface_param.floor_level ]; % z value of the desired base position is updated.
        end
        
        %%% Timing
        gait_planning_param.base_T = [time; time + gait_planning_param.T/2];
        
        %%% Set the half point for feasibility check
        gait_planning_param.base_next_midpoint =  (gait_planning_param.base_next +  gait_planning_param.base_cur)/2;
        
    %%% When The walking sequence is updated and changed %%%
    elseif gait_planning_param.crawl_gait_sequence_change_flag == true
        %%% To keep TSM >= 0, stop swinging limb once, and moved base COM to the intersection of two diagonal lines
        % The swing limb number was set as the last number of the new
        % sequence in upd_walking_sequence_based_on_moving_direction.
        % This is because the first swing leg of the sequence is selected
        % in the next period, since the swing leg is the next index in the
        % previous sequence. 
        
        % Set the Next desired positions of limbs
        gait_planning_param.POS_next = POS_e; % Not move limbs
        % Set the next base pos based on the intersection of
        % diagonal lines
        gait_planning_param = upd_base_next_pos_CoM_projection_on_intersection_of_diagonals(SV,des_SV,LP,gait_planning_param,POS_e,inc,surface_param,base_height,time);
        
    end
end
end

