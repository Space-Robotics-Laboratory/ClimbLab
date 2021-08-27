%%%%%% Update
%%%%%% upd_kinematic_feasibility
%%%%%%
%%%%%% Update next base position in the way that the CoM projection is put
%%%%%% on the intersection of the two diagonal lines
%%%%%%
%%%%%% Created: 2020-11-18
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-07-06
%%%%%% Keigo Haji
%
%
% You can see the details of the planning method is shown in the following paper:
% --------------------------------------------------------------------
% In: Proceedings of the IEEE/SICE International Symposium on System 
% Integration (SII) 2019 by K. Uno et al.
% Proceedings Paper URL: 
% https://ieeexplore.ieee.org/document/8700455
% --------------------------------------------------------------------
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%
%         gait_planning_param.kinematic_feasibility_check_is_OK : 1 or 0
%     INPUT
%         SV                    : State values (SpaceDyn class
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         gait_planning_param   : Parameters for gait planning (class)

function gait_planning_param = upd_kinematic_feasibility(SV, des_SV, LP, POS_e, gait_planning_param)
% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb

RequirementCheck_CNT = 0;
i = gait_planning_param.swing_number;

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Requirement Check of Supporting Leg flag
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for k = 1:LP.num_limb + 1
        
        if k <= 4 % k = 1 -- 4 for current supporting points 
            pos_next = POS_e(:,k);
            if k == i && gait_planning_param.feasibility_check_number_of_legs == 4 
                % When gait_planning_param.feasibility_check_number_of_legs is set as 4.
                % Do not check if the swing leg can grasp the point that it is currently grasping after it moves. 
                continue;
            end
        elseif k == 5 % k == 5 for one next grasping point for the swing limb
            k = i;
            pos_next = gait_planning_param.POS_next(:,k);
        end
                      
        diff1 = abs( LP.reachable_area.data_far(3,:) - ( pos_next(3,1)-SV.R0(3) ) );
        diff2 = abs( LP.reachable_area.data_near(3,:) - ( pos_next(3,1)-SV.R0(3) ) );
        [~,I_1] = min(diff1(:));
        [~,I_2] = min(diff2(:));
        %         x1 = LP.reachable_area.data_far(2,I_1(1));%         x2 = LP.reachable_area.data_near(2,I_2(1));%         MaxRange = x1;%         MinRange = x2;%         MinRange_from_Joint = MinRange - LP.c0(1,1);%         MaxRange_from_Joint = MaxRange - LP.c0(1,1);
        MinRange_from_Joint = LP.reachable_area.data_near(2,I_2(1)) - LP.c0(1,1);
        MaxRange_from_Joint = LP.reachable_area.data_far(2,I_1(1)) - LP.c0(1,1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        if contains(gait_planning_param.type, 'crawl') ...
                && (gait_planning_param.swing_number == gait_planning_param.sequence(1) || gait_planning_param.swing_number == gait_planning_param.sequence(3))
            p_check = pos_next + ( - ( gait_planning_param.base_next_midpoint - SV.R0 ) );
        else
            p_check = pos_next + ( - ( gait_planning_param.base_next - SV.R0 ) );
        end
        
%         p_check = pos_next + ( - ( gait_planning_param.base_next - SV.R0 ) );
        p_from_CoM = p_check(:) - SV.R0;
        %     p_from_CoM = POS_e(:,k) - gait_planning_param.base_next;
%         p_from_Joint = p_from_CoM - LP.c0(:, 3*k-2);
%         p_from_Joint = p_from_CoM - SV.A0*LP.c0(:, 3*k-2);
        p_from_Joint = p_from_CoM - rpy2dc(gait_planning_param.base_orientation_next)' * LP.c0(:, 3*k-2);
        d = norm( p_from_Joint(1:2) );
        % ------------------The following is previous codes ---------------
        %         % <<< Step 1 >>>
        %         if d < MaxRange_from_Joint && d > MinRange_from_Joint
        %             % <<< Step 2 >>>
        %             theta = - SV.Q0(3); % initialize theta with offsetting the yaw angle at this moment
        %             theta = theta + atan2(p_from_Joint(2), p_from_Joint(1));
        %             switch k %leg-number
        %                 case 1
        %                 case 2
        %                     if theta <= 0
        %                         theta = theta + 2*pi;
        %                     end
        %                 case 3
        %                     if theta <= 0
        %                         theta = theta + 2*pi;
        %                     end
        %                 case 4
        %                     theta = theta + 2*pi;
        %             end
        %             if theta > ( LP.Qi(3, (3*k-2)) + deg2rad(LP.joint_limit(1,1)) ) && ...
        %                theta < ( LP.Qi(3, (3*k-2)) + deg2rad(LP.joint_limit(1,2)) )
        %                 RequirementCheck_CNT = RequirementCheck_CNT + 1;
        %             else
        %     %             flag = 0;
        %                 break;
        %             end
        %         else
        %     %         flag = 0;
        %             break;
        %         end
        % -----------------------------------------------------------------
        % <<< Step 1 >>>
        if d < MaxRange_from_Joint && d > MinRange_from_Joint
            % <<< Step 2 >>>
            Base2limbCoxa = rpy2dc(gait_planning_param.base_orientation_next)'*LP.c0(:,3*k-2);
            dot_tmp = dot(p_from_Joint(1:2), Base2limbCoxa(1:2));
            theta = acos(dot_tmp/d/norm(Base2limbCoxa(1:2)));   % 0 to pi            
            cross_tmp = cross([p_from_Joint(1),p_from_Joint(2),0], [Base2limbCoxa(1),Base2limbCoxa(2),0]);
            if cross_tmp(3) > 0
                if theta < abs(LP.joint_limit(1,1)*pi/180)
                RequirementCheck_CNT = RequirementCheck_CNT + 1;
                end
            elseif cross_tmp(3) <= 0                
                if theta < abs(LP.joint_limit(1,2)*pi/180)               
                RequirementCheck_CNT = RequirementCheck_CNT + 1;
                end
            else
    %             flag = 0;
                break;
            end
        else
    %         flag = 0;
            break;
        end
    end
    
    if gait_planning_param.feasibility_check_number_of_legs == 5 && RequirementCheck_CNT == 5 % four current supporting points + one next grasping point for the swing limb
        gait_planning_param.kinematic_feasibility_check_is_OK = true;
    elseif gait_planning_param.feasibility_check_number_of_legs == 4 && RequirementCheck_CNT == 4 % three current supporting points + one next grasping point for the swing limb
        gait_planning_param.kinematic_feasibility_check_is_OK = true;
    else
        gait_planning_param.kinematic_feasibility_check_is_OK = false;
    end

    %% If the next desired landing point is same as last time (i.e. the 
    %  norm of the distance is too small), it should be removed from the
    %  options.
    if strcmp(gait_planning_param.feasibility_check_excluding_same_graspable_point_switch, 'on')
    if norm(gait_planning_param.POS_next(:, i)-POS_e(:, i)) <= 0.005 % this threshold can be smaller
        gait_planning_param.kinematic_feasibility_check_is_OK = false;
    end
    end

    %% If the next desired landing point is same as other legs (i.e. the 
    %  norm of the distance is too small), it should be removed from the
    %  options.
    for j = 1:LP.num_limb
        if norm(gait_planning_param.POS_next(:, i)-POS_e(:, j)) <= 0.005 
                if j ~= i
                gait_planning_param.kinematic_feasibility_check_is_OK = false;
                end
        end
    end
    
    %% If the last solution did not pass the feasibility check, the option is removed.
    if ~gait_planning_param.kinematic_feasibility_check_is_OK
        % to substitute the null into the matrix, we cannnot use three
        % dimensional matrix --> we should prepare them for each limb
        switch i
            case 1
                gait_planning_param.LF_graspable_points_in_reachable_area(:, gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride) = [nan,nan,nan];
            case 2
                gait_planning_param.LH_graspable_points_in_reachable_area(:, gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride) = [nan,nan,nan];
            case 3
                gait_planning_param.RH_graspable_points_in_reachable_area(:, gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride) = [nan,nan,nan];
            case 4
                gait_planning_param.RF_graspable_points_in_reachable_area(:, gait_planning_param.index_of_graspable_points_in_reachable_area_to_allow_max_stride) = [nan,nan,nan];
        end
    end
end
end