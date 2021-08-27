%%%%%% Initialize
%%%%%% ini_id
%%%%%% 
%%%%%% Create id for naming
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last updated: 2021-03-31
%%%%%% Keigo Haji
%
%
% Create identification for file naming
%
%     Function variables:
%
%     OUTPUT
%         run_id              : Run identification (string)
%         run_date            : Run identification date (string)
%     INPUT
%         robot_param         : Parameter for robot (class)
%         environment_param   : Parameter for environment (class)
%         gait_planning_param : Parameter for gait planning (class)
%         control_param       : Parameter for control (class)

function [run_id, run_date] = ini_id(robot_param, environment_param, gait_planning_param, control_param)

run_date = [datestr(now,'yymmdd_HHMMSS')]; % Date and time
% run_id = [run_date '-' robot_param.robot_type '-' gait_planning_param.type '-' control_param.type '-' environment_param.surface_type ...
%           '-' num2str(environment_param.inc) '[deg]-' num2str(environment_param.grav) '[G]'];
run_id = [run_date];

end