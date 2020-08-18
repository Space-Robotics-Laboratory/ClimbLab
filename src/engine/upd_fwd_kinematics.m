%%%%%% Update
%%%%%% upd_fwd_kinematics
%%%%%% 
%%%%%% Calculate forward kinematics
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-09
%
% Use SpaceDyn functions for forward kinematics to obtain the position and orientation of the end-effectors. Also converts
% the base orientation to degrees
%
% Function variables:
%
%     OUTPUT
%         POS_e        : Position of the end-effector [m] (3xnum_limb matrix)
%         Qe_deg       : Orientation of the end-effector (Euler angles) [deg] (3xnum_limb matrix)
%         Q0_deg       : Orientation of the base (Euler angles) [deg] (3x1 vector)
%     INPUT
%         LP           : Link Parameters (SpaceDyn class)
%         SV           : State Variables (SpaceDyn class)

function [POS_e, Qe_deg, Q0_deg] = upd_fwd_kinematics(LP, SV)

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