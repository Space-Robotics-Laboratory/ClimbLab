%%%%%% Initialize
%%%%%% ini_id
%%%%%% 
%%%%%% Create video file
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Create identification for file naming
%
%     Function variables:
%
%     OUTPUT
%         run_id          : Run identification (string)
%         run_date        : Run identification date (string)
%     INPUT
%         robot_type      : Robot type (string) 
%         surface_type    : Surface type (string) 
%         grav            : Gravity [G] (scalar) 
%         inc             : Surface inclination [deg] (scalar) 
%         gait_param.type : Gait type (string) 

function [run_id, run_date] = ini_id(robot_param, environment_param, gait_param, control_param)

run_date = [datestr(now,'yymmdd_HHMMSS')]; % Date and time
run_id = [run_date '-' robot_param.robot_type '-' gait_param.type '-' control_param.type '-' environment_param.surface_type ...
          '-' num2str(environment_param.inc) '[deg]-' num2str(environment_param.grav) '[G]'];

end