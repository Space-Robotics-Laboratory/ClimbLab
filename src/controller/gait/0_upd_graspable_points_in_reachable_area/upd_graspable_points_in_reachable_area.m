%%%%%% Update
%%%%%% upd_graspable_points_in_reachable_area
%%%%%%
%%%%%% Update the graspable points in reachable area
%%%%%%
%%%%%% Created 2020-05-18
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-05-19
%%%%%% Keigo Haji
%
%
% Update the graspable points in reachable area for gait planning
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param                   : Parameters for gait planning (class)
%
%         gait_planning_param.graspable_points_in_reachable_area: graspable
%         points in the four limbs" reachable region
%
%     INPUT
%         SV                                    : State values (SpaceDyn class)
%         des_SV                                : Desired state values (SpaceDyn class)
%         LP                                    : Link parameters (SpaceDyn class)
%         surface_param.graspable_points        : 3xN matrix of the graspable points in the map [m]
%         surface_param.sensed_graspable_points : 3xN matrix of the sensed graspable points in the map [m]
%                                                 For the points that have not yet been sensed, the corresponding columns hold the NaN values.
%         gait_planning_param                   : Parameters for path planning (class)
%         sensing_camera_param                  : Parameters for sensing camera (class)


function [gait_planning_param] = upd_graspable_points_in_reachable_area(SV,des_SV,LP,surface_param,gait_planning_param, sensing_camera_param)
% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb

% judge the graspable point_i is reachable by leg-i
p_from_Joint = zeros(3,1);
% prepare the array to store the graspable points in reachable area [[points], n, Limb number ]
gait_planning_param.graspable_points_in_reachable_area = [];

% choose whether all grasped points are known or walk while sensing.
switch sensing_camera_param.sensing_flag
    case 'off'
        % Position of all the graspable points on the map relative to Base
        graspable_points(3,:) = surface_param.graspable_points(3,:) - SV.R0(3,1);
        
    case 'on'
        % Position of only sensed graspable points relative to Base
        graspable_points(3,:) = surface_param.sensed_graspable_points(3,:) - SV.R0(3,1);
end

% get an index of the swing leg
for i = 1 : 1 : LP.num_limb
    count = 1;
    for k = 1 : length(surface_param.graspable_points)
    
        diff1 = abs( LP.reachable_area.data_far(3,:) - graspable_points(3,k));
        diff2 = abs( LP.reachable_area.data_near(3,:) - graspable_points(3,k));
        % M_n: Min. value, I_n: the index
        % *see also the usage of min() for more info.)
        [~,I_1] = min(diff1(:));
        [~,I_2] = min(diff2(:));
    
        if I_1(1)
            x1 = LP.reachable_area.data_far(2,I_1(1));
            y1 = LP.reachable_area.data_far(3,I_1(1));
        end
        if I_2(1)
            x2 = LP.reachable_area.data_near(2,I_2(1));
            y2 = LP.reachable_area.data_near(3,I_2(1));
        end
    
        % MaxRange and MinRange are defined by the height of surface_param.graspable_points
        % respectively. Note that surface_param.graspable_points's height is defined in the way
        % that robot CoM is height = zero.
        MaxRange = x1;
        MinRange = x2;
        MinRange_from_Joint = MinRange - LP.c0(1,1);
        MaxRange_from_Joint = MaxRange - LP.c0(1,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % choose whether all grasped points are known or walk while sensing
        switch sensing_camera_param.sensing_flag
            case 'off'
                p_from_Joint = surface_param.graspable_points(:,k) - (SV.R0 + SV.A0*LP.c0(:,3*i-2));
            case 'on'
                p_from_Joint = surface_param.sensed_graspable_points(:,k) - (SV.R0 + SV.A0*LP.c0(:,3*i-2));
        end
        
        d = norm(p_from_Joint(1:2));
        % ------------------The following is previous codes ---------------
        % <<< Step 1 >>>
        %         if d < MaxRange_from_Joint && d > MinRange_from_Joint
        %             % <<< Step 2 >>>
        %             theta = - SV.Q0(3); % initialize theta with offsetting the yaw angle at this moment
        %             theta = theta + atan2(p_from_Joint(2), p_from_Joint(1));
        %             switch i %leg-number
        %                 case 1
        %                 case 2
        %                     if theta <= 0
        %                         theta = theta + 2*pi;
        %                     end
        %                 case 3
        %
        %                     if theta <= 0
        %                         theta = theta + 2*pi;
        %                     end
        %                 case 4
        %                     theta = theta + 2*pi;
        %             end
        %             if theta > LP.Qi(3, (3*i-2)) + LP.joint_limit(1,1)*pi/180 && theta < LP.Qi(3, (3*i-2)) + LP.joint_limit(1,2)*pi/180
        %                 gait_planning_param.graspable_points_in_reachable_area(:,count,i) = [surface_param.graspable_points(1,k); surface_param.graspable_points(2,k); surface_param.graspable_points(3,k)];
        %                 count = count + 1;
        %                 % plot3(surface_param.graspable_points(1,k), surface_param.graspable_points(2,k), surface_param.graspable_points(3,k),'.r',...
        %                 %     'MarkerEdgeColor',[0.0 1.0 0.0],'MarkerSize',10)
        %             end
        %         end
        % -----------------------------------------------------------------
        if d < MaxRange_from_Joint && d > MinRange_from_Joint
            % <<< Step 2 >>>
            Base2limbCoxa = SV.A0*LP.c0(:,3*i-2);
            dot_tmp = dot(p_from_Joint(1:2), Base2limbCoxa(1:2));
            theta = acos(dot_tmp/d/norm(Base2limbCoxa(1:2)));   % 0 to pi
            cross_tmp = cross([p_from_Joint(1),p_from_Joint(2),0], [Base2limbCoxa(1),Base2limbCoxa(2),0]);
            if cross_tmp(3) > 0
                if theta < abs(LP.joint_limit(1,1)*pi/180)
                    gait_planning_param.graspable_points_in_reachable_area(:,count,i) = [surface_param.graspable_points(1,k); surface_param.graspable_points(2,k); surface_param.graspable_points(3,k)];
                    count = count + 1;
                end
            elseif  cross_tmp(3) <= 0
                if theta < abs(LP.joint_limit(1,2)*pi/180)
                    gait_planning_param.graspable_points_in_reachable_area(:,count,i) = [surface_param.graspable_points(1,k); surface_param.graspable_points(2,k); surface_param.graspable_points(3,k)];
                    count = count + 1;
                end
            end
        end

    end

end
end

