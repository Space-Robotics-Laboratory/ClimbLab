%%%%%% Initialize
%%%%%% ini_robot
%%%%%% 
%%%%%% Initialize robot parameters
%%%%%% 
%%%%%% Created 2019-10-30
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-11 by Kentaro Uno
%
%
% Initialize robot's links parameters and state variables, including gripping force and end-effector positions and
% orientations
%
% Function variables:
%
%     OUTPUT
%         LP           : Link Parameters (SpaceDyn class)
%         des_SV       : Desired State Variables (SpaceDyn class)
%         SV           : State Variables (SpaceDyn class)
%         F_grip       : Maximum gripping force [N] (scalar)
%         POS_e        : Position of the end-effector [m] (3xnum_limb matrix)
%         ORI_e        : Orientation of the end-effector [DC] (3x3*num_limb matrix)
%         cont_POS     : Initial point of contact which is used as equilibrium position [m] (3xnum_limb matrix)
%     INPUT
%         robot        : Robot type (string) 
%         foot_dist    : Distance from projection of robot center to end-point in both x and y directions [m] (scalar)
%         base_height  : Height of base from mean surface of the ground [m] (scalar)
%         gait_param   : Gait parameters
%         surface_param: Surface parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% New variables for LP and SV, other than what is explained by SpaceDyn documents
%
%         LP.num_limb  : number of limbs (scalar)
%         SV.sup       : vector to indicate if the i-th limb is a supporting limb (SV.sup(i) = 1) or a swinging limb 
%                        (SV.sup(i) = 0) (1xn vector)
%         SV.slip      : vector to indicate if the i-th limb is slipping (SV.slip(i) = 1) or not (SV.slip(i) = 0)(1xn vector)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LP, SV, des_SV, F_grip, POS_e, ORI_e, cont_POS] = ini_robot(robot_param, gait_param, surface_param)

global contact_f
global x ; global y ; global z;

switch robot_param.robot_type

case 'HubRobo_no_grip'
    LP = ini_HubRobo_no_grip_LP();
    F_grip = 0.0;
    % Calculate reachable area
    joint_limit=[-60 60; 0 90; -135 0]; %[deg]
    
case 'HubRobo_grip_to_spine'
    LP = ini_HubRobo_grip_to_spine_LP();
    F_grip = 3.2;
    % Calculate reachable area
    joint_limit=[-60 60; 0 90; -135 0]; %[deg]
    
case 'HubRobo_grip_to_spine_old'
    LP = ini_HubRobo_grip_to_spine_LP_old();
    F_grip = 3.2;
    % Calculate reachable area
    joint_limit=[-60 60; 0 90; -135 0]; %[deg]
    
case 'HubRobo_grip_to_palm'
    LP = ini_HubRobo_grip_to_palm_LP();
    F_grip = 3.2;
    % Calculate reachable area
    joint_limit=[-60 60; 0 90; -135 0]; %[deg]
        
otherwise
    disp('invalid robot type');

end

LP.num_limb = sum(LP.SE); % Total number of limbs
contact_f = zeros(1,LP.num_limb); % Limb end-effector contact flag

%%% State variables initialization %%%
SV = ini_12DOF_SV( LP );                                                         
% Initial Base position
if isnumeric(robot_param.base_pos_xy)
    SV.R0 = [robot_param.base_pos_xy(1:2,1); robot_param.base_height + surface_param.floor_level ];
else
    SV.R0 = [x(round(length(x)/2)) ;  y(round(length(y)/2)) ; robot_param.base_height + surface_param.floor_level ] ; % center of the map
end
% Supporting limb
SV.sup = zeros(1,LP.num_limb); % Supporting limb flag
SV.slip = zeros(1,LP.num_limb);% Supporting limb slip flag 
% Initialize supporting limbs
SV.sup = [1 1 1 1];

% Set initial joint angle [rad] 
foot_dist = robot_param.foot_dist;
SV.q = ini_joint_angle(LP,SV,foot_dist);    

%%% Links positions %%%
SV = calc_aa( LP, SV );
SV = calc_pos( LP, SV );

joints = zeros(LP.num_limb,3);
POS_e = zeros(3,LP.num_limb);
ORI_e = zeros(3,3*LP.num_limb);
for i = 1:LP.num_limb
    % Joints/links connecting the base to each limb end-effector
    joints(i,:) = j_num(LP, i);
    % Each limb end-effector pos/ori  [m] [DCM]
    [POS_e(:,i), ORI_e(:,3*i-2 : 3*i)] = f_kin_e(LP, SV, joints(i,:));
end
% Initialize contact position
cont_POS = POS_e;

% Initialize desired state
des_SV = SV;
if strcmp(gait_param.type,'crawl_uno_ver')
    LP = ini_reachable_area(LP,SV, POS_e(3,1)-SV.R0(3,1),joint_limit);
end
end