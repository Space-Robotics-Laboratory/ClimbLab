%%%%%% Initialize
%%%%%% ini_Climbing_ANYmal_LP
%%%%%% 
%%%%%% Mass/inertia parameters and link connections for possible climbing
%%%%%% ANYmal robot model
%%%%%% 
%%%%%% --------------------------------------------------------------------
%%%%%% This configration file can reproduce the simulation result used in
%%%%%% CLAWAR 2021 proceedings paper by K. Uno and G. Valsecchi et al.
%%%%%% Proceedings Paper URL: 
%%%%%% https://******** (to be added)
%%%%%% --------------------------------------------------------------------
%%%%%% 
%%%%%% 
%%%%%% Created: 2021-02-02
%%%%%% by Kentaro Uno
%%%%%% Last update: 2021-08-24
%%%%%% by Kentaro Uno

function [LP, ani_settings] = ini_Climbing_ANYmal_LP(robot_param, ani_settings)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Copied from the optimized values    %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% My robot
%%% Add my robot properties for climbing and concurrent design %%%
robot.myRobot.mass.total = 22.8; % only used to calculated cost of transport
robot.myRobot.legCount   = 4;

% Density of each link
% kg/m^3. Density values calculated to give correct link mass when link
% approximated as solid cylinder.
robot.myRobot.legDensity.hip(1)       = 9728.3;   robot.myRobot.legDensity.hip(2)     = 9728.3;
robot.myRobot.legDensity.thigh(1)     = 5826.3;   robot.myRobot.legDensity.thigh(2)   = 5826.3;
robot.myRobot.legDensity.shank(1)     = 888.2668; robot.myRobot.legDensity.shank(2)     = 888.2668;
robot.myRobot.legDensity.foot(1)      = 800;      robot.myRobot.legDensity.foot(2)      = 800;
robot.myRobot.legDensity.phalanges(1) = 800;      robot.myRobot.legDensity.phalanges(2) = 800;    

% End effector mass
robot.myRobot.EE(1).mass = 0.1402;
robot.myRobot.EE(2).mass = 0.1402;

% Offset from nominal CoM position to base hip attachment for each leg.
robot.myRobot.xNom(1) = 0.05;
robot.myRobot.xNom(2) = 0.05;
robot.myRobot.yNom(1) = 0.14;
robot.myRobot.yNom(2) = 0.11;
robot.myRobot.zNom = 0.01; % offset from CoM to HAA in z direction. Positive value means HAA above CoM.

robot.myRobot.nomHipPos.LF = [ robot.myRobot.xNom(1),  robot.myRobot.yNom(1), robot.myRobot.zNom];
robot.myRobot.nomHipPos.LH = [-robot.myRobot.xNom(2),  robot.myRobot.yNom(2), robot.myRobot.zNom];
robot.myRobot.nomHipPos.RF = [ robot.myRobot.xNom(1), -robot.myRobot.yNom(1), robot.myRobot.zNom];
robot.myRobot.nomHipPos.RH = [-robot.myRobot.xNom(2), -robot.myRobot.yNom(2), robot.myRobot.zNom];

% redefine the Offset from nominal CoM position to base hip attachment for each leg based on the CAD.
robot.myRobot.nomHipPos.LF = [ 0.15,  0.15, 0.0];
robot.myRobot.nomHipPos.LH = [-0.15,  0.15, 0.0];
robot.myRobot.nomHipPos.RF = [ 0.15, -0.15, 0.0];
robot.myRobot.nomHipPos.RH = [-0.15, -0.15, 0.0];

% Link lengths [m]
robot.myRobot.hip(1).length   = 0.100; % updated with the optimized value.
robot.myRobot.hip(2).length   = 0.100; % updated with the optimized value.
robot.myRobot.thigh(1).length = 0.434; % updated with the optimized value.
robot.myRobot.thigh(2).length = 0.434; % updated with the optimized value.
robot.myRobot.shank(1).length = 0.418; % updated with the optimized value. 
robot.myRobot.shank(2).length = 0.418; % updated with the optimized value.  
robot.myRobot.foot(1).length  = 0.16; % for now, not used
robot.myRobot.foot(2).length  = 0.16; % for now, not used
robot.myRobot.phalanges(1).length = 0.1; % for now, not used
robot.myRobot.phalanges(2).length = 0.1; % for now, not used

% Link radius [m] (only used to calculate link mass as solid cylinder)
robot.myRobot.hip(1).radius = 0.015;
robot.myRobot.hip(2).radius = 0.015;
robot.myRobot.thigh(1).radius = 0.015;
robot.myRobot.thigh(2).radius = 0.015;
robot.myRobot.shank(1).radius = 0.015;
robot.myRobot.shank(2).radius = 0.015;
robot.myRobot.foot(1).radius = 0.015;
robot.myRobot.foot(2).radius = 0.015;
robot.myRobot.phalanges(1).radius = 0.015;
robot.myRobot.phalanges(2).radius = 0.015;

% Transmission Ratio
% For remote and directly actuated joints
% GearRatio = input speed/ output speed
% Front legs                                        Hind legs
robot.myRobot.transmissionGearRatio.HAA(1) = 5.6;  robot.myRobot.transmissionGearRatio.HAA(2) = 5.6;
robot.myRobot.transmissionGearRatio.HFE(1) = 5.6;  robot.myRobot.transmissionGearRatio.HFE(2) = 5.6;
robot.myRobot.transmissionGearRatio.KFE(1) = 5.6;  robot.myRobot.transmissionGearRatio.KFE(2) = 5.6;
robot.myRobot.transmissionGearRatio.AFE(1) = 5.6;  robot.myRobot.transmissionGearRatio.AFE(2) = 5.6;
robot.myRobot.transmissionGearRatio.DFE(1) = 5.6;  robot.myRobot.transmissionGearRatio.DFE(2) = 5.6;    

% Joint angle limits (used only in reachable positions plot)
% q1 HAA, q2 HFE, q3 KFE, q4 AFE
robot.myRobot.q1.minAngle = -pi;
robot.myRobot.q1.maxAngle = pi;
robot.myRobot.q2.minAngle = -2*pi;
robot.myRobot.q2.maxAngle = 2*pi;
robot.myRobot.q3.minAngle = -2*pi;
robot.myRobot.q3.maxAngle = 2*pi;
robot.myRobot.q4.minAngle = -pi;
robot.myRobot.q4.maxAngle = pi;
robot.myRobot.q5.minAngle = -pi;
robot.myRobot.q5.maxAngle = pi;

% Base dimensions used for visualization - visualized as a box
robot.myRobot.baseLength = 0.3;
robot.myRobot.baseWidth  = 0.3;
robot.myRobot.baseHeight = 0.15;     

    %% newly added parameters necessary for climbing/concurrent design
    
    % Nominal EE positions, translation from base to EE
%     robot.myRobot.nomBaseHeight = 0.42;
%     robot.myRobot.nomEEPos.LF = [  0.34,  0.19, -robot.myRobot.nomBaseHeight];
%     robot.myRobot.nomEEPos.RF = [  0.34, -0.19, -robot.myRobot.nomBaseHeight];
%     robot.myRobot.nomEEPos.LH = [ -0.34,  0.19, -robot.myRobot.nomBaseHeight];
%     robot.myRobot.nomEEPos.RH = [ -0.34, -0.19, -robot.myRobot.nomBaseHeight];

    % maximum deviation from nominal EE position (edge length of foot workspace is equal to 2*maxDeviationEE = 2*b)
    robot.myRobot.maxDeviationEEInitial.LF = [0.15; 0.1; 0.1];
    robot.myRobot.maxDeviationEEInitial.RF = [0.15; 0.1; 0.1];
    robot.myRobot.maxDeviationEEInitial.LH = [0.15; 0.1; 0.1];
    robot.myRobot.maxDeviationEEInitial.RH = [0.15; 0.1; 0.1];
    
    % maximum EE gripping force
    robot.myRobot.grippingForce = 200;
    
    % dynamic model robot, according to TOWR for ANYmal
%     Ixx =  0.946438;
%     Iyy =  1.94478;
%     Izz =  2.01835;
%     Ixy =  0.000938112;
%     Ixz = -0.00595386;
%     Iyz = -0.00146328;

    % dynamic model robot
    Ixx = 0.05618;
    Iyy = 0.06743;
    Izz = 0.05055;
    Ixy = 0.014655;
    Ixz = 0.0945;
    Iyz = 0.0945;
    
    % merging tensor (no user taks)
    robot.myRobot.inertiaTensor = [ Ixx, -Ixy, -Ixz; 
                                   -Ixy,  Iyy, -Iyz; 
                                   -Ixz, -Iyz,  Izz]; % inertia Tensor of base
                               
     %% added parameters necessary for ClimbLab
     % note that z axis is always rotation axis, and x axis is always towards the EE in ClimbLab
     % LF hip link inertial tensor calculation 
     robot.myRobot.inertiaTensorForLinks.LF.hip.mass = robot.myRobot.legDensity.hip(1) * pi ... 
                                                       * robot.myRobot.hip(1).length * robot.myRobot.hip(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.hip.Ixx = 1/2 * robot.myRobot.inertiaTensorForLinks.LF.hip.mass ...
                                                          * robot.myRobot.hip(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.hip.Iyy = 1/4 * robot.myRobot.inertiaTensorForLinks.LF.hip.mass ...
                                                          * robot.myRobot.hip(1).radius^2 ...
                                                    + 1/12 * robot.myRobot.inertiaTensorForLinks.LF.hip.mass ...
                                                           * robot.myRobot.hip(1).length^2;
     robot.myRobot.inertiaTensorForLinks.LF.hip.Izz = robot.myRobot.inertiaTensorForLinks.LF.hip.Iyy;
     % LF thigh link inertial tensor calculation 
     robot.myRobot.inertiaTensorForLinks.LF.thigh.mass = robot.myRobot.legDensity.thigh(1) * pi ... 
                                                       * robot.myRobot.thigh(1).length * robot.myRobot.thigh(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.thigh.Ixx = 1/2 * robot.myRobot.inertiaTensorForLinks.LF.thigh.mass ...
                                                          * robot.myRobot.thigh(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.thigh.Iyy = 1/4 * robot.myRobot.inertiaTensorForLinks.LF.thigh.mass ...
                                                          * robot.myRobot.thigh(1).radius^2 ...
                                                    + 1/12 * robot.myRobot.inertiaTensorForLinks.LF.thigh.mass ...
                                                           * robot.myRobot.thigh(1).length^2;
     robot.myRobot.inertiaTensorForLinks.LF.thigh.Izz = robot.myRobot.inertiaTensorForLinks.LF.thigh.Iyy;
     % LF shank link inertial tensor calculation 
     robot.myRobot.inertiaTensorForLinks.LF.shank.mass = robot.myRobot.legDensity.shank(1) * pi ... 
                                                       * robot.myRobot.shank(1).length * robot.myRobot.shank(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.shank.Ixx = 1/2 * robot.myRobot.inertiaTensorForLinks.LF.shank.mass ...
                                                          * robot.myRobot.shank(1).radius^2;
     robot.myRobot.inertiaTensorForLinks.LF.shank.Iyy = 1/4 * robot.myRobot.inertiaTensorForLinks.LF.shank.mass ...
                                                          * robot.myRobot.shank(1).radius^2 ...
                                                    + 1/12 * robot.myRobot.inertiaTensorForLinks.LF.shank.mass ...
                                                           * robot.myRobot.shank(1).length^2;
     robot.myRobot.inertiaTensorForLinks.LF.shank.Izz = robot.myRobot.inertiaTensorForLinks.LF.shank.Iyy;
     
     % all limb has the same inertial spec.
     robot.myRobot.inertiaTensorForLinks.LH = robot.myRobot.inertiaTensorForLinks.LF;
     robot.myRobot.inertiaTensorForLinks.RH = robot.myRobot.inertiaTensorForLinks.LF;
     robot.myRobot.inertiaTensorForLinks.RF = robot.myRobot.inertiaTensorForLinks.LF;
     

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  Visualization settings are changed here based on the above values  %%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Link radius [m]
ani_settings.link_radius = 2*robot.myRobot.hip(1).radius; % drawn radius is bigger than phisic model. same radius for all links
%%% Base thickness [m]
ani_settings.base_upper_thickness = 2/3 * robot.myRobot.baseHeight;
ani_settings.base_lower_thickness = 1/3 * robot.myRobot.baseHeight;
%%% Base horisontal scaling factor
ani_settings.base_xy_scale_factor = 1.0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% From here, the LP used in ClimbLab are substitued by with the abone values. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% nomally, the LP should be defined here.
LP.joint_allocation_type = 'mammal'; % state the type of the joint configuration -> IK analytical solver is switched
% LP.leg_config_type = 'oo'; % if the joint alloc. is mammalian, you need to decide the leg configuration as well

% local variables to define the offset angle theta_1 and theta_2
LP.theta_1 = 0.1440; % [rad]
LP.theta_2 = -0.1955; % [rad]

if isnan(robot_param.theta_1) == 0  % this if sentence is only applied when you set the value in config file
    LP.theta_1 = robot_param.theta_1;
end
if isnan(robot_param.theta_2) == 0  % this if sentence is only applied when you set the value in config file
    LP.theta_2 = robot_param.theta_2;
end
% this if sentence is only for the special alias to define the theta1 and 2 in config file
if strcmp(robot_param.joint_allocation_type, 'insect') 
    % The following offset angle setting enables the robot model to be spider
    % configuration
    LP.theta_1 = pi/4; % [rad]
    LP.theta_2 = -pi/2; % [rad]
end

if robot_param.leg_config_type ~= "" % this if sentence is only for the special switch in config file
    LP.leg_config_type = robot_param.leg_config_type;
end


% Max. endurable gripping force
LP.F_grip = robot.myRobot.grippingForce;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% Definition of each link parameters %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Define link connection relationship
LP.BB = [ 0 1 2 0 4 5 0 7 8 0 10 11];

% Number of links/joints
LP.num_q = length(LP.BB); 

LP.SS = [ -1  1  0  0  0  0  0  0  0  0  0  0;
           0 -1  1  0  0  0  0  0  0  0  0  0;
           0  0 -1  0  0  0  0  0  0  0  0  0;
           0  0  0 -1  1  0  0  0  0  0  0  0;
           0  0  0  0 -1  1  0  0  0  0  0  0;
           0  0  0  0  0 -1  0  0  0  0  0  0;
           0  0  0  0  0  0 -1  1  0  0  0  0;
           0  0  0  0  0  0  0 -1  1  0  0  0;
           0  0  0  0  0  0  0  0 -1  0  0  0;
           0  0  0  0  0  0  0  0  0 -1  1  0;
           0  0  0  0  0  0  0  0  0  0 -1  1;
           0  0  0  0  0  0  0  0  0  0  0 -1];
% Links connected to base (link 0)
LP.S0 = [ 1 0 0 1 0 0 1 0 0 1 0 0];
% End link (leg)
LP.SE = [ 0 0 1 0 0 1 0 0 1 0 0 1 ];
% Type of joint
LP.J_type = [ 'R' 'R' 'R'  'R' 'R' 'R'  'R' 'R' 'R'  'R' 'R' 'R' ];
% Movable limitation of joint [deg]
LP.joint_limit=[rad2deg(robot.myRobot.q1.minAngle) rad2deg(robot.myRobot.q1.maxAngle); % HAA
                rad2deg(robot.myRobot.q2.minAngle) rad2deg(robot.myRobot.q2.maxAngle); % HFE
                rad2deg(robot.myRobot.q3.minAngle) rad2deg(robot.myRobot.q3.maxAngle)]; % KFE

% Position vector from base CoM to i-th joint. Order is LF_HAA, LF_HFE, 
% LF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE, RF_HAA, RF_HFE, and RF_KFE.  
LP.c0 = [  robot.myRobot.nomHipPos.LF(1)  0  0  robot.myRobot.nomHipPos.LH(1)  0  0 robot.myRobot.nomHipPos.RH(1)  0  0  robot.myRobot.nomHipPos.RF(1)  0  0;
           robot.myRobot.nomHipPos.LF(2)  0  0  robot.myRobot.nomHipPos.LH(2)  0  0 robot.myRobot.nomHipPos.RH(2)  0  0  robot.myRobot.nomHipPos.RF(2)  0  0;
           robot.myRobot.nomHipPos.LF(3)  0  0  robot.myRobot.nomHipPos.LH(3)  0  0 robot.myRobot.nomHipPos.RH(3)  0  0  robot.myRobot.nomHipPos.RF(3)  0  0];
% Mass of the base (link 0) <- copied from URDF file 
LP.m0 = 16.793507758; % [kg] 
% Moment of inertia of base <- copied from URDF file
LP.inertia0 = robot.myRobot.inertiaTensor;

% Mass of each link <- based on manual measurement
LP.m = [ robot.myRobot.inertiaTensorForLinks.LF.hip.mass robot.myRobot.inertiaTensorForLinks.LF.thigh.mass robot.myRobot.inertiaTensorForLinks.LF.shank.mass + robot.myRobot.EE(1).mass...
         robot.myRobot.inertiaTensorForLinks.LH.hip.mass robot.myRobot.inertiaTensorForLinks.LH.thigh.mass robot.myRobot.inertiaTensorForLinks.LH.shank.mass + robot.myRobot.EE(1).mass...
         robot.myRobot.inertiaTensorForLinks.RH.hip.mass robot.myRobot.inertiaTensorForLinks.RH.thigh.mass robot.myRobot.inertiaTensorForLinks.RH.shank.mass + robot.myRobot.EE(1).mass...
         robot.myRobot.inertiaTensorForLinks.RF.hip.mass robot.myRobot.inertiaTensorForLinks.RF.thigh.mass robot.myRobot.inertiaTensorForLinks.RF.shank.mass + robot.myRobot.EE(1).mass ]; % [kg]
% Total mass 
LP.mass = sum(LP.m) + LP.m0; % [kg]

%%% Rotational relationship of links frames

% so, with the rotation about theta_1 around the z axis, then rotation
% about theta_2+pi/2 around the y' axis, the hip joint frame is obtained.
% However, for the SpaceDyn function, we should describe this in not z-y-x
% euler angle, but in x-y-z euler angle!!
LF_euler_angle_ZYX = [LP.theta_1 pi/2+LP.theta_2 0];  LH_euler_angle_ZYX = [-LP.theta_1 pi/2-LP.theta_2 0];
RH_euler_angle_ZYX = [LP.theta_1 pi/2-LP.theta_2 0];  RF_euler_angle_ZYX = [-LP.theta_1 pi/2+LP.theta_2 0];
LF_rotation_matrix = eul2rotm(LF_euler_angle_ZYX,'ZYX');
LH_rotation_matrix = eul2rotm(LH_euler_angle_ZYX,'ZYX');
RH_rotation_matrix = eul2rotm(RH_euler_angle_ZYX,'ZYX');
RF_rotation_matrix = eul2rotm(RF_euler_angle_ZYX,'ZYX');

LF_euler_angle_XYZ = rotm2eul(LF_rotation_matrix,'XYZ');
LH_euler_angle_XYZ = rotm2eul(LH_rotation_matrix,'XYZ');
RH_euler_angle_XYZ = rotm2eul(RH_rotation_matrix,'XYZ');
RF_euler_angle_XYZ = rotm2eul(RF_rotation_matrix,'XYZ');

LP.Qi = [  LF_euler_angle_XYZ(1)  pi/2  0  LH_euler_angle_XYZ(1)   pi/2  0  RH_euler_angle_XYZ(1)  pi/2  0  RF_euler_angle_XYZ(1)   pi/2  0;
           LF_euler_angle_XYZ(2)  0     0  LH_euler_angle_XYZ(2)   0     0  RH_euler_angle_XYZ(2)  0     0  RF_euler_angle_XYZ(2)   0     0;
           LF_euler_angle_XYZ(3)  0     0  LH_euler_angle_XYZ(3)   0     0  RH_euler_angle_XYZ(3)  0     0  RF_euler_angle_XYZ(3)   0     0];

% Inertia tensor of each link
LP.inertia = [ robot.myRobot.inertiaTensorForLinks.LF.hip.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.LF.thigh.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.LF.shank.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.LH.hip.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.LH.thigh.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.LH.shank.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RH.hip.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RH.thigh.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RH.shank.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RF.hip.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RF.thigh.Ixx 0 0    robot.myRobot.inertiaTensorForLinks.RF.shank.Ixx 0 0 ;
               0 robot.myRobot.inertiaTensorForLinks.LF.hip.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.LF.thigh.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.LF.shank.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.LH.hip.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.LH.thigh.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.LH.shank.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RH.hip.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RH.thigh.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RH.shank.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RF.hip.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RF.thigh.Iyy 0    0 robot.myRobot.inertiaTensorForLinks.RF.shank.Iyy 0 ;
               0 0 robot.myRobot.inertiaTensorForLinks.LF.hip.Izz    0 0 robot.myRobot.inertiaTensorForLinks.LF.thigh.Izz    0 0 robot.myRobot.inertiaTensorForLinks.LF.shank.Izz    0 0 robot.myRobot.inertiaTensorForLinks.LH.hip.Izz    0 0 robot.myRobot.inertiaTensorForLinks.LH.thigh.Izz    0 0 robot.myRobot.inertiaTensorForLinks.LH.shank.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RH.hip.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RH.thigh.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RH.shank.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RF.hip.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RF.thigh.Izz    0 0 robot.myRobot.inertiaTensorForLinks.RF.shank.Izz ];

% Position vector from each link CoM to i-th joint
LP.cc = zeros( 3,12,12 ); 

% Postion vector from link 1 CoM to joint 1 seen from the joint 1 frame
LP.cc(:,1,1) = [  0          -0         -robot.myRobot.hip(1).length/2 ]';
% Postion vector from link 1 CoM to joint 2 
LP.cc(:,1,2) = [  0           0          robot.myRobot.hip(1).length/2 ]';
LP.cc(:,2,2) = [ -robot.myRobot.thigh(1).length/2       0            0 ]';
LP.cc(:,2,3) = [  robot.myRobot.thigh(1).length/2       0           -0 ]';
LP.cc(:,3,3) = [ -robot.myRobot.shank(1).length/2       0            0 ]'; 

LP.cc(:,4,4) = [  0          -0          robot.myRobot.hip(1).length/2 ]';
LP.cc(:,4,5) = [  0           0         -robot.myRobot.hip(1).length/2 ]';
LP.cc(:,5,5) = [ -robot.myRobot.thigh(1).length/2       0            0 ]';
LP.cc(:,5,6) = [  robot.myRobot.thigh(1).length/2       0           -0 ]';
LP.cc(:,6,6) = [ -robot.myRobot.shank(1).length/2       0            0 ]'; 

LP.cc(:,7,7) = [  0           0          robot.myRobot.hip(1).length/2 ]';
LP.cc(:,7,8) = [  0          -0         -robot.myRobot.hip(1).length/2 ]';
LP.cc(:,8,8) = [ -robot.myRobot.thigh(1).length/2       0           -0 ]';
LP.cc(:,8,9) = [  robot.myRobot.thigh(1).length/2       0            0 ]';
LP.cc(:,9,9) = [ -robot.myRobot.shank(1).length/2       0            0 ]'; 

LP.cc(:,10,10) = [  0           0       -robot.myRobot.hip(1).length/2 ]';
LP.cc(:,10,11) = [  0          -0        robot.myRobot.hip(1).length/2 ]';
LP.cc(:,11,11) = [ -robot.myRobot.thigh(1).length/2      0          -0 ]';
LP.cc(:,11,12) = [  robot.myRobot.thigh(1).length/2      0           0 ]';
LP.cc(:,12,12) = [ -robot.myRobot.shank(1).length/2      0           0 ]'; 

% Position vector from end link CoM to end point
LP.ce = [  0  0  0.32125/2 0  0  0.32125/2        0  0  0.32125/2          0  0  0.32125/2  ;
           0  0  0         0  0 -0                0  0 -0                  0  0  0          ;
           0  0  0         0  0  0                0  0  0                  0  0  0         ];
% Rotational relationship of end link frames
LP.Qe = [  0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0;
           0  0  0  0  0  0  0  0  0  0  0  0 ];

% EOF