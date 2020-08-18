%%%%%% Update
%%%%%% upd_graspable_points_in_reachable_area
%%%%%%
%%%%%% Update the graspable points in reachable area
%%%%%%
%%%%%% Created 2020-05-18
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-05-18
%
%
% Update the graspable points in reachable area for path planning
%
% Function variables:
%
%     OUTPUT
%         path_planning_param                   : Parameters for path planning (class)
%
%
%     INPUT
%         SV                                    : State values (SpaceDyn class)
%         LP                                    : Link parameters (SpaceDyn class)
%         surface_param.graspable_points        : 3xN matrix of the graspable points in the map [m]
%         path_planning_param                   : Parameters for path planning (class)

function [path_planning_param] = upd_graspable_points_in_reachable_area(SV,LP,surface_param,path_planning_param)
% judge the graspable point_i is reachable by leg-i
p_from_Joint = zeros(3,1);
path_planning_param.graspable_points_in_reachable_area=[];

% get an index of the swing leg
i = path_planning_param.swing_number;

% Position of graspable points relative to Base
graspable_points(3,:) = surface_param.graspable_points(3,:) - SV.R0(3,1);

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
    
    p_from_Joint = surface_param.graspable_points(:,k) - (SV.R0 + SV.A0*LP.c0(:,3*i-2));
    d = norm(p_from_Joint(1:2));
    % <<< Step 1 >>>
    if d < MaxRange_from_Joint && d > MinRange_from_Joint
        % <<< Step 2 >>>
        theta = atan2(p_from_Joint(2), p_from_Joint(1));
        switch i %leg-number
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
        if theta > LP.Qi(3, (3*i-2)) + LP.joint_limit(1,1)*pi/180 && theta < LP.Qi(3, (3*i-2)) + LP.joint_limit(1,2)*pi/180
            path_planning_param.graspable_points_in_reachable_area = horzcat(path_planning_param.graspable_points_in_reachable_area, [surface_param.graspable_points(1,k); surface_param.graspable_points(2,k); surface_param.graspable_points(3,k)]);
            % plot3(surface_param.graspable_points(1,k), surface_param.graspable_points(2,k), surface_param.graspable_points(3,k),'.r',...
            %     'MarkerEdgeColor',[0.0 1.0 0.0],'MarkerSize',10)
        end
    end
end

end

