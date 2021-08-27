%%%%%% Visualize
%%%%%% vis_sensing_camera_fov
%%%%%%
%%%%%% Visualize the fov of a RealSense camera
%%%%%%
%%%%%% Created: 2021-02-10
%%%%%% by Keigo Haji
%%%%%% Last update: 2021-03-03
%%%%%% by Keigo Haji
%
%
% Visualize the fov of RealSense camera
%
% Function variables:
%
%     OUTPUT
%       -
%
%     INPUT
%       SV                      : State Variables (SpaceDyn class)
%       inc                     : Surface inclination [deg] (scalar)
%       sensing_camera_param    : Setting the fov of the camera
%           mounting_angle      : Mounting angle of the camera to the base
%           mounting_position   : Mounting position of the camera on the base
%           fov_horizontal      : Horizontal angle of view of the camera
%           fov_vertical        : Vertical angle of view of the camera
%           fov_max_distance    : Maximum distance that depth can be detected
%           fov_min_distance    : Minimum distance that depth can be detected
%       ani_settings            : Setting the line width and color for drawing the fov
%

function vis_sensing_camera_fov(SV, inc, sensing_camera_param, ani_settings, surface_param)

if strcmp(ani_settings.sensing_fov_show, 'on')
    % Rotation matrix
    rot = rpy2dc([0;pi*inc/180;0])';
    if strcmp(sensing_camera_param.sensing_type, 'RealSense_d435i')
      
        % Update camera position
        camera.pos = SV.R0 + SV.A0 * sensing_camera_param.mounting_position;
        
        
        % Set the camera range for the far side
        far_y_minus = sensing_camera_param.fov_max_distance * tan(-sensing_camera_param.fov_horizontal * 0.5); % < 0
        far_y_plus  = sensing_camera_param.fov_max_distance * tan( sensing_camera_param.fov_horizontal * 0.5); % > 0
        far_z_minus = sensing_camera_param.fov_max_distance * tan(-sensing_camera_param.fov_vertical * 0.5); % < 0
        far_z_plus  = sensing_camera_param.fov_max_distance * tan( sensing_camera_param.fov_vertical * 0.5); % > 0
        % Set the camera range for the near side
        near_y_minus = sensing_camera_param.fov_min_distance * tan(-sensing_camera_param.fov_horizontal * 0.5); % < 0
        near_y_plus  = sensing_camera_param.fov_min_distance * tan( sensing_camera_param.fov_horizontal * 0.5); % > 0
        near_z_minus = sensing_camera_param.fov_min_distance * tan(-sensing_camera_param.fov_vertical * 0.5); % < 0
        near_z_plus  = sensing_camera_param.fov_min_distance * tan( sensing_camera_param.fov_vertical * 0.5); % > 0
        
        
        % Set the coordinates of the end points that form the fov with respect
        % to the camera coordinate system. The four points on the near side and
        % the four points on the far side are set separately.
        % Each row represents an xyz coordinate, and each column represents a
        % point.
        camera.max_scan_range =...
            [sensing_camera_param.fov_max_distance sensing_camera_param.fov_max_distance sensing_camera_param.fov_max_distance sensing_camera_param.fov_max_distance ;
            far_y_minus                           far_y_plus                            far_y_plus                            far_y_minus             ;
            far_z_plus                            far_z_plus                            far_z_minus                           far_z_minus            ];
        camera.min_scan_range =...
            [sensing_camera_param.fov_min_distance sensing_camera_param.fov_min_distance sensing_camera_param.fov_min_distance sensing_camera_param.fov_min_distance ;
            near_y_minus                          near_y_plus                           near_y_plus                           near_y_minus             ;
            near_z_plus                           near_z_plus                           near_z_minus                          near_z_minus            ];
        
        
        % Transform coordinates from a camera-based coordinate system to a
        % ground-based coordinate system.
        camera.max_scan_range = camera.pos ...
            + SV.A0 * rpy2dc(sensing_camera_param.mounting_angle) * camera.max_scan_range;
        camera.min_scan_range = camera.pos ...
            + SV.A0 * rpy2dc(sensing_camera_param.mounting_angle) * camera.min_scan_range;
        
        % Transform coordinates from the ground coordinate system to the
        % inertial coordinate system.
        camera_pos = rot' * camera.pos;
        max_scan_range = rot' * camera.max_scan_range;
        min_scan_range = rot' * camera.min_scan_range;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Visualize %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Draw the camera mounting position
        plot3(camera_pos(1),camera_pos(2),camera_pos(3), ani_settings.sensing_camera_MarkerColor,'MarkerSize',ani_settings.sensing_camera_MarkerSize);
        
        % Draw the end points forming fov
        plot3(max_scan_range(1,:),max_scan_range(2,:),max_scan_range(3,:), ani_settings.sensing_camera_fov_LineColor,'LineWidth', ani_settings.sensing_camera_fov_LineWidth);
        plot3(min_scan_range(1,:),min_scan_range(2,:),min_scan_range(3,:), ani_settings.sensing_camera_fov_LineColor,'LineWidth', ani_settings.sensing_camera_fov_LineWidth);
        
        % Draw two rectangles connecting the end points
        extended_matrix_of_max_scan_range_for_plotting = [max_scan_range max_scan_range(:,1)];
        plot3(extended_matrix_of_max_scan_range_for_plotting(1,:), extended_matrix_of_max_scan_range_for_plotting(2,:), extended_matrix_of_max_scan_range_for_plotting(3,:),ani_settings.sensing_camera_fov_LineColor,'LineWidth', ani_settings.sensing_camera_fov_LineWidth)
        extended_matrix_of_min_scan_range_for_plotting = [min_scan_range min_scan_range(:,1)];
        plot3(extended_matrix_of_min_scan_range_for_plotting(1,:), extended_matrix_of_min_scan_range_for_plotting(2,:), extended_matrix_of_min_scan_range_for_plotting(3,:),ani_settings.sensing_camera_fov_LineColor,'LineWidth', ani_settings.sensing_camera_fov_LineWidth)
        
        % Draw four trapezoids connecting the near end-points and far
        % end-points
        for i = 1:4
            plot3([min_scan_range(1,i) max_scan_range(1,i)],[min_scan_range(2,i) max_scan_range(2,i)],[min_scan_range(3,i) max_scan_range(3,i)], ani_settings.sensing_camera_fov_LineColor,'LineWidth', ani_settings.sensing_camera_fov_LineWidth)
        end
        
        if strcmp(ani_settings.sensing_fov_face_filling, 'on')
            % Colors fov
            patch([min_scan_range(1,1) max_scan_range(1,1) max_scan_range(1,2),min_scan_range(1,2)],[min_scan_range(2,1) max_scan_range(2,1) max_scan_range(2,2),min_scan_range(2,2)],[min_scan_range(3,1) max_scan_range(3,1) max_scan_range(3,2),min_scan_range(3,2)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
            patch([min_scan_range(1,2) max_scan_range(1,2) max_scan_range(1,3),min_scan_range(1,3)],[min_scan_range(2,2) max_scan_range(2,2) max_scan_range(2,3),min_scan_range(2,3)],[min_scan_range(3,2) max_scan_range(3,2) max_scan_range(3,3),min_scan_range(3,3)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
            patch([min_scan_range(1,4) max_scan_range(1,4) max_scan_range(1,3),min_scan_range(1,3)],[min_scan_range(2,4) max_scan_range(2,4) max_scan_range(2,3),min_scan_range(2,3)],[min_scan_range(3,4) max_scan_range(3,4) max_scan_range(3,3),min_scan_range(3,3)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
            patch([min_scan_range(1,1) max_scan_range(1,1) max_scan_range(1,4),min_scan_range(1,4)],[min_scan_range(2,1) max_scan_range(2,1) max_scan_range(2,4),min_scan_range(2,4)],[min_scan_range(3,1) max_scan_range(3,1) max_scan_range(3,4),min_scan_range(3,4)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
            patch([min_scan_range(1,1) min_scan_range(1,2) min_scan_range(1,3),min_scan_range(1,4)],[min_scan_range(2,1) min_scan_range(2,2) min_scan_range(2,3),min_scan_range(2,4)],[min_scan_range(3,1) min_scan_range(3,2) min_scan_range(3,3),min_scan_range(3,4)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
            patch([max_scan_range(1,1) max_scan_range(1,2) max_scan_range(1,3),max_scan_range(1,4)],[max_scan_range(2,1) max_scan_range(2,2) max_scan_range(2,3),max_scan_range(2,4)],[max_scan_range(3,1) max_scan_range(3,2) max_scan_range(3,3),max_scan_range(3,4)],ani_settings.sensing_fov_face_color,'FaceAlpha',ani_settings.sensing_fov_face_alpha, 'EdgeColor', ani_settings.sensing_camera_fov_LineColor);
        end
    end
    
    if strcmp(sensing_camera_param.sensing_type, 'circle')
        
        r = sensing_camera_param.circular_radius_from_base_pos;        
        t = linspace(0, 2*pi, 50);
        
        % Arc
        X1 = [r*cos(t) ; r*sin(t) ; ones(1,length(t)) * surface_param.floor_level];
        X2 = [r*cos(-t) ; r*sin(-t) ; ones(1,length(t))*surface_param.floor_level];
        
        % Move origin
        X1 = X1+[SV.R0(1:2);0];
        X2 = X2+[SV.R0(1:2);0];
        
        % Adjust inclination
        X1 = rot' * X1;
        X2 = rot' * X2;
        
        % Plot
        plot3([X1(1,:) X2(1,:) X1(1,1)],[X1(2,:) X2(2,:) X1(2,1)],[X1(3,:) X2(3,:) X1(3,1)]+0.005,'Color',ani_settings.sensing_camera_fov_LineColor,'LineWidth',ani_settings.sensing_camera_fov_LineWidth);
        
        
        
    end
end
end

