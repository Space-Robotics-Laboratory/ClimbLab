%%%%%% Update
%%%%%% upd_swing_num_and_grasping_point_based_on_graspable_options
%%%%%% 
%%%%%% Update number of swing leg based on how many graspable points the
%%%%%% swing limb has in its reachable area in forward 
%%%%%% 
%%%%%% Created: 2020-07-02
%%%%%% Kentaro Uno
%%%%%% Last updated: 2021-05-04
%%%%%% Keigo Haji
%
%
% You can see the details of the planning method is shown in the following paper:
% --------------------------------------------------------------------
% In: Proceedings of the International Symposium on Artificial Intelligence
% Robotics and Automation in Space 
% Integration (iSAIRAS) 2020 by K. Uno et al.
% Proceedings Paper URL: 
%%%%%% https://www.hou.usra.edu/meetings/isairas2020fullpapers/pdf/5027.pdf
% --------------------------------------------------------------------
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for path planning (class)
%
%         gait_planning_param.swing_number      : Number of the selected leg to be the next swing leg (scalar)
%
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         surface_param         : Parameters for surface (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         inc                   : Surface inclination [deg] (scalar) 
%         base_height           : Height of base relative to map [m]
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_swing_num_based_on_num_of_graspable_options(gait_planning_param, des_SV, SV, LP, POS_e, surface_param)
if sum(des_SV.sup) == LP.num_limb

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Preparation of matrix that contains GP in RA, limb code, and score
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Graspable points in Reachable area
    GP_in_RA = [];
%     % Graspable Points in Reachable Area in front 
%     GP_in_RA_in_front = [];
    % integrated matrix that contains [ [GP_in_RA_in_front]; limbCode; score ], where score is currently used dot product
    GP_in_RA_in_front_with_limbCode_and_score = [];
    dot_tmp = [];
    count = 1;
	
    for i = 1 : 1 : LP.num_limb
        % get the number of the graspable points which is located in front from the current position of the limb in its reachable area 

        % Current position of swing leg on the surface
        [ gait_planning_param.POS_cur(1,i), ...
          gait_planning_param.POS_cur(2,i), ...
          gait_planning_param.POS_cur(3,i) ] = get_map_pos_of_graspable_points(POS_e(1,i),POS_e(2,i),surface_param);

        % Current position of the base
        gait_planning_param.base_cur = SV.R0;
        
        % Graspable points in Reachable area for limb i
        switch i
            case 1
                GP_in_RA(:,:,i) = gait_planning_param.LF_graspable_points_in_reachable_area;
            case 2
                GP_in_RA(:,:,i) = gait_planning_param.LH_graspable_points_in_reachable_area;
            case 3
                GP_in_RA(:,:,i) = gait_planning_param.RH_graspable_points_in_reachable_area;
            case 4
                GP_in_RA(:,:,i) = gait_planning_param.RF_graspable_points_in_reachable_area;
        end        
        
        % Vector from Base position to goal
        vec_bg = gait_planning_param.goal(:,gait_planning_param.goal_num) - SV.R0;
    
        for ind = 1:size(GP_in_RA(:,:,i),2)
            vec_tmp = GP_in_RA(:,ind,i) - POS_e(:,i);
            % too much long swing is excluded out from the possible next
            % gripping points
            if norm(vec_tmp) <= 0.2
                dot_tmp(1,ind,i) = dot(vec_bg(:,1), vec_tmp);
                % if the following dot product is positive, the GP is located in
                % front of this swing leg
                if dot_tmp(1,ind,i) > 0
                    %GP_in_RA_in_front(:,count,i) = GP_in_RA(:,ind,i);
                    GP_in_RA_in_front_with_limbCode_and_score(:,count) = [GP_in_RA(:,ind,i);i;dot_tmp(1,ind,i)];
                    count = count + 1;             
                end
            else
                dot_tmp(1,ind,i) = 0;
            end
        end    
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Select new swing limb
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % decide the limb that has most graspable points in reachable area in
    % front as the rentative swing limb --> next gripping points selection
    gait_planning_param.num_of_graspable_points_in_front = [0;0;0;0];
    for i = 1 : 1 : LP.num_limb
        for j = 1 : size(GP_in_RA_in_front_with_limbCode_and_score(:,:),2)
            if GP_in_RA_in_front_with_limbCode_and_score(4,j) == i
                gait_planning_param.num_of_graspable_points_in_front(i) = gait_planning_param.num_of_graspable_points_in_front(i) + 1;
            end
        end
    end
    [~,I] = max(gait_planning_param.num_of_graspable_points_in_front);
    gait_planning_param.swing_number = I;
    
end
    
end