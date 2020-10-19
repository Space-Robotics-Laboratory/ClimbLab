%%%%%% Update
%%%%%% upd_fwd_dynamics
%%%%%% 
%%%%%% Calculate forward dynamics
%%%%%% 
%%%%%% Created 2020-04-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-15
%
% Use SpaceDyn functions for forward dynamics to obtain joints and links accelerations, velocities and positions from the
% given torques and external forces
%
% Function variables:
%
%     OUTPUT
%         SV                  : State Variables (SpaceDyn class)
%     INPUT 
%		  environment_param.dynamics_flag   : Flag to compute ('on') or not compute ('off') dynamics
%         LP                  : Link parameters (SpaceDyn class)
%         SV                  : State variables (SpaceDyn class)
%         des_SV              : Desired state variables (SpaceDyn class)

function [POS_e, Qe_deg, Q0_deg, SV] = upd_fwd_kinematics(LP, SV)

POS_e = zeros(3,LP.num_limb);
Qe_rad = zeros(3,LP.num_limb);
Qe_deg = zeros(3,LP.num_limb);

% Calculate each end-effector position from joint angle
for i = 1:LP.num_limb
    joints = j_num(LP, i);
	% Each limb end-effector pos/ori  [m] [DCM]
    [POS_e(:,i), ORI_e(:,3*i-2 : 3*i)] = f_kin_e(LP, SV, joints);
    % Limb end-effector orientation in Euler angles
	Qe_rad(:,i) = dc2rpy( ORI_e(:,3*i-2 : 3*i)' ); % [rad]
    Qe_deg(:,i) = 180/pi*Qe_rad(:,i);              % [deg]
end

% Base orientation in Euler angles
SV.Q0 = dc2rpy( SV.A0' ); % [rad]
Q0_deg = 180/pi*SV.Q0;    % [deg]

end