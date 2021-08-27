%%%%%% Initialize
%%%%%% ini_graspable_points
%%%%%% 
%%%%%% Extract graspable points from mesh
%%%%%% 
%%%%%% Created 2020-05-12
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2021-04 -16
%%%%%% Keigo Haji
%
%
% Extract graspable points according to the graspable_points_detection_type
%
% Function variables:
%
%     OUTPUT
%         surface_param.graspable_points   : 3xN matrix contains position vectors of graspable points [m] (x;y;z)
%     INPUT
%         environment_param   : Parameters for environment (class)
%         surface_param       : Parameters for surface (class)
%         gripper_param
%         map_param
%         matching_settings
%     USGAE EXAMPLE
%         environment_param.graspable_points_detection_type = 50;
%         surface_param = ini_graspable_points(graspable_points_detection_type, surface_param, gripper_param, map_param, matching_setting);
%         environment_param.graspable_points_detection_type = 'gripper'; 

function surface_param = ini_graspable_points(environment_param, surface_param, gripper_param, map_param, matching_settings)
global x;global y;global z;

[n,m]=size(z);
surface_param.graspable_points=zeros(3,n*m);
[X,Y] = meshgrid(x,y);
surface_param.graspable_points(1,:)=reshape(X,1,n*m);
surface_param.graspable_points(2,:)=reshape(Y,1,n*m);
surface_param.graspable_points(3,:)=reshape(z,1,n*m);

% Deletes the columns that have NaN values.  
surface_param.graspable_points = rmmissing(surface_param.graspable_points, 2);
% set the floor level variable to have a z directional height in global
% frame.    
surface_param.floor_level = mean(surface_param.graspable_points(3,:));


switch environment_param.graspable_points_detection_type
    case 'all'
        ;
    case 'gripper'
        surface_param.graspable_points = target_detection(surface_param.graspable_points, environment_param.graspable_points_detection_type, gripper_param, map_param, matching_settings);
    case 'peaks'
        surface_param.graspable_points = target_detection(surface_param.graspable_points, environment_param.graspable_points_detection_type, gripper_param, map_param, matching_settings);
    case 'climbing_holds_map_on_testfield'
        load('graspable_points_of_testfield_without_board.mat');
        surface_param.graspable_points = graspable_points_on_testfield_deleted_targets_on_board;
        % load('graspable_points_on_test_field.mat');
        % surface_param.graspable_points = graspable_points_on_test_field;
    case 'climbing_holds_map_on_full_testfield'
        load('graspable_points_of_full_testfield.mat');
        surface_param.graspable_points = graspable_points_on_full_testfield;
        % load('graspable_points_on_test_field.mat');
        % surface_param.graspable_points = graspable_points_on_test_field;
    otherwise
        if isnumeric(environment_param.graspable_points_detection_type)
            if environment_param.graspable_points_detection_type >=0 && environment_param.graspable_points_detection_type <=100
                cnt = uint64( n * m * environment_param.graspable_points_detection_type/100 );
                while cnt > 0 && ~isempty(surface_param.graspable_points)
                    rm = randi(size(surface_param.graspable_points,2)) ;
                    surface_param.graspable_points(:,rm) = []; % remove the point from surface_param.graspable_points.
                    cnt = cnt - 1;
                end
            end
        end
end
end
