%%%%%% Update
%%%%%% upd_swing_next_pos_crawl_uno_ver
%%%%%%
%%%%%% Update next grasping position for crawl gait
%%%%%%
%%%%%% Created 2020-06-12
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-09-11 by Kentaro Uno
%
%
% Update next grasping position for crawl gait using Uno-san's method
%
% Function variables:
%
%     OUTPUT
%         path_planning_param   : Parameters for path planning (class)
%
%         path_planning_param.POS_cur       : Current position of the swing leg when selecting a new one [m] (3x1 vector)
%         path_planning_param.POS_next      : Next desired position of the swing leg [m] (3x1 vector)
%         path_planning_param.base_next     : Next desired position of the base [m] (3x1 vector)
%         path_planning_param.leg_T         : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%         path_planning_param.base_T        : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         path_planning_param   : Parameters for path planning (class)
%         gait_param            : Parameters for gait (class)
%         surface_param         : Parameters for surface (class)
%         SV                    : State values (SpaceDyn class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         inc                   : Surface inclination [deg] (scalar) 
%         base_height           : Height of base relative to map [m]
%         time                  : Simulation time [s] (scalar)

function path_planning_param = upd_swing_next_pos_crawl_uno_ver(path_planning_param, gait_param, surface_param, SV, des_SV, LP, POS_e, inc, base_height, time)
if sum(des_SV.sup) == LP.num_limb && ~isempty(path_planning_param.graspable_points_in_reachable_area)
    i = path_planning_param.swing_number;
    % Current position of swing leg on the surface
    [path_planning_param.POS_cur(1,i),path_planning_param.POS_cur(2,i),...
        path_planning_param.POS_cur(3,i)] = get_map_pos(POS_e(1,i),POS_e(2,i));

    % Current position of the base
    path_planning_param.base_cur = SV.R0;
    % Graspable points in Reachable area
    GP_in_RA = [];
    GP_in_RA = path_planning_param.graspable_points_in_reachable_area(:,:,i);
    % Projection point (current position in a map)
    [~,~,c] =  get_map_pos(SV.R0(1),SV.R0(2));
    %     Base_proj = [a;b;c];
    Base_proj = [SV.R0(1:2,1);c];
    
    % Vector from CoM projecton point to goal
    vec_sg = gait_param.goal - Base_proj;
    %     vec_sg = vec_sg/norm(vec_sg);
    
    dot_tmp = zeros(1,size(GP_in_RA,2));
    path_planning_param.POS_next = POS_e;
    for ind = 1:size(GP_in_RA,2)
        vec_tmp = GP_in_RA(1:2,ind) - POS_e(1:2,i);
        if norm(vec_tmp) <= 0.2
            dot_tmp(1,ind) = dot(vec_sg(1:2,1), vec_tmp);
        else
            dot_tmp(1,ind) = 0;
        end
    end
    
    RequirementCheck_flag = false;
    path_planning_param.POS_last_selected(:, i) = path_planning_param.POS_next(:, i);
    while ~RequirementCheck_flag
        [~,I] = max(dot_tmp);
        %         disp([GP_in_RA;dot_tmp]);
        path_planning_param.POS_next(:, i) = GP_in_RA(:,I);
        path_planning_param.base_next = Base_Pos(SV,LP,path_planning_param,POS_e,inc,surface_param,base_height);
        
        %         plot3(GP_in_RA(1,:),GP_in_RA(2,:),GP_in_RA(3,:),'.','MarkerSize',20);
        %         disp('Next gripping point is :')
        %         disp([GP_in_RA(:,I);dot_tmp(:,I)]);
        
        RequirementCheck_flag = true;
        %         RequirementCheck_flag = Requirement_Check(RequirementCheck_flag,LP,path_planning_param);
        RequirementCheck_flag = Requirement_Check(SV,LP,POS_e,path_planning_param);
        % Also, if the next desired landing point is same as last time, it should
        % be removed from the options
        if norm(path_planning_param.POS_next(:, i)-POS_e(:, i)) <= 0.01
            RequirementCheck_flag = false
        end
        
        if ~RequirementCheck_flag
            GP_in_RA(:, I) = [];
            dot_tmp(I) = [];
        end
        
        if isempty(GP_in_RA)
            %         disp('best next gripping point does not exist!!!');
            path_planning_param.POS_next(:,i) = POS_e(:,i);
            path_planning_param.base_next = SV.R0;
            break;
        end
    end
    % Time for current and desired positions
    path_planning_param.leg_T = [time; time + gait_param.T_d];
    % Timing
    path_planning_param.base_T = [time; time + gait_param.T_d];
    

    % Record the footholds of leg i
    if time == 0
        for j=1:LP.num_limb
            path_planning_param.footholds_history_limb(:,path_planning_param.footholds_count_limb(j),j) = POS_e(:,j);
            path_planning_param.footholds_count_limb(j) = path_planning_param.footholds_count_limb(j) + 1;
        end
    end
    path_planning_param.footholds_history_limb(:,path_planning_param.footholds_count_limb(i),i) = path_planning_param.POS_next(:,i);
    path_planning_param.footholds_count_limb(i) = path_planning_param.footholds_count_limb(i) + 1;

end
end

function [ Base_Position ] = Base_Pos(SV,LP,path_planning_param,POS_e,inc,surface_param,base_height)
Base_Position = SV.R0;

L1_x = zeros(2,1);
L1_y = zeros(2,1);
L2_x = zeros(2,1);
L2_y = zeros(2,1);

for j = 1:LP.num_limb
    if j == path_planning_param.swing_number
        switch j
            case 1
                L1_x(1) = path_planning_param.POS_next(1,j);
                L1_y(1) = path_planning_param.POS_next(2,j);
            case 2
                L2_x(1) = path_planning_param.POS_next(1,j);
                L2_y(1) = path_planning_param.POS_next(2,j);
            case 3
                L1_x(2) = path_planning_param.POS_next(1,j);
                L1_y(2) = path_planning_param.POS_next(2,j);
            case 4
                L2_x(2) = path_planning_param.POS_next(1,j);
                L2_y(2) = path_planning_param.POS_next(2,j);
        end
    else
        switch j
            case 1
                L1_x(1) = POS_e(1,j);
                L1_y(1) = POS_e(2,j);
            case 2
                L2_x(1) = POS_e(1,j);
                L2_y(1) = POS_e(2,j);
            case 3
                L1_x(2) = POS_e(1,j);
                L1_y(2) = POS_e(2,j);
            case 4
                L2_x(2) = POS_e(1,j);
                L2_y(2) = POS_e(2,j);
        end
    end
end

% Calculate distances
Dx12 = L1_x(1)-L1_x(2);
Dx34 = L2_x(1)-L2_x(2);
Dy12 = L1_y(1)-L1_y(2);
Dy34 = L2_y(1)-L2_y(2);
Dx24 = L1_x(2)-L2_x(2);
Dy24 = L1_y(2)-L2_y(2);
% Solve the equation
ts = [Dx12 -Dx34; Dy12 -Dy34] \ [-Dx24; -Dy24];
% Take weighted combinations of points on the line
P = ts(1)*[L1_x(1); L1_y(1)] + (1-ts(1))*[L1_x(2); L1_y(2)];

% figure();hold on;grid on;
% plot(L1_x,L1_y);
% plot(L2_x,L2_y);
% plot(P(1),P(2),'ko');
% plot(POS_e(1,:),POS_e(2,:),'bo');
Base_Position(1,1) = P(1) + norm(surface_param.floor_level)*tan(inc*pi/180);
Base_Position(2,1) = P(2);
Base_Position(3,1) = base_height + surface_param.floor_level;
end

function flag = Requirement_Check(SV,LP,POS_e,path_planning_param)
RequirementCheck_CNT = 0;
% Requirement Check of Supporting Leg flag
for k = 1:LP.num_limb
    if path_planning_param.swing_number == k
        pos_next = path_planning_param.POS_next(:,k);
    else
        pos_next = POS_e(:,k);
    end
    pos_next(3,1) = pos_next(3,1) - SV.R0(3);
    diff1 = abs( LP.reachable_area.data_far(3,:) - pos_next(3,1) );
    diff2 = abs( LP.reachable_area.data_near(3,:) - pos_next(3,1) );
    [~,I_1] = min(diff1(:));
    [~,I_2] = min(diff2(:));
    %         x1 = LP.reachable_area.data_far(2,I_1(1));%         x2 = LP.reachable_area.data_near(2,I_2(1));%         MaxRange = x1;%         MinRange = x2;%         MinRange_from_Joint = MinRange - LP.c0(1,1);%         MaxRange_from_Joint = MaxRange - LP.c0(1,1);
    MinRange_from_Joint = LP.reachable_area.data_near(2,I_2(1)) - LP.c0(1,1);
    MaxRange_from_Joint = LP.reachable_area.data_far(2,I_1(1)) - LP.c0(1,1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_check = pos_next + ( - ( path_planning_param.base_next - SV.R0 ) );
    p_from_CoM = p_check(:) - SV.R0;
    %     p_from_CoM = POS_e(:,k) - path_planning_param.base_next;
    p_from_Joint = p_from_CoM - LP.c0(:, 3*k-2);
    d = norm( p_from_Joint(1:2,1));
    % <<< Step 1 >>>
    if d < MaxRange_from_Joint && d > MinRange_from_Joint
        % <<< Step 2 >>>
        theta = atan2(p_from_Joint(2), p_from_Joint(1));
        switch k %leg-number
            case 1
            case 2
                if theta <= 0
                    theta = theta + 2*pi;
                end
            case 3
                if theta <= 0
                    theta = theta + 2*pi;
                end
            case 4
                theta = theta + 2*pi;
        end
        if theta > LP.Qi(3, (3*k-2)) - pi/3 && theta < LP.Qi(3, (3*k-2)) + pi/3
            RequirementCheck_CNT = RequirementCheck_CNT + 1;
        else
%             flag = 0;
            break;
        end
    else
%         flag = 0;
        break;
    end
end
if RequirementCheck_CNT == 4
    flag = true;
else
    flag = false;
end
end