%%%%%% Initialize
%%%%%% ini_environment
%%%%%% 
%%%%%% Initialize simulation parameters
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-09
%
%
% Initialize global environment parameters
%
% Function variables:
%
%     OUTPUT
%         ---
%     INPUT
%         environment_param   : Parameters for environment (class)


function ini_environment(environment_param)

global d_time
global Gravity
global Ez

% Global variables
Gravity = rpy2dc([0;pi*environment_param.inc/180;0])'*environment_param.grav*[0 0 -9.81]'; % Gravity vector [m/s^2]

Ez = [0 0 1]';  % Unit vector for joints rotation axis

d_time = environment_param.time_step; % Time-step [s]