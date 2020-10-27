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

function SV = upd_fwd_dynamics(environment_param, LP, SV, des_SV)

global d_time;

if strcmp(environment_param.dynamics_flag,'on')
	% Dynamics on
	% Solve equation of motion
	SV = f_dyn_rk2(LP,SV);
	SV = f_dyn(LP,SV);
else
	% Dynamics off
	des_SV.v0 = (des_SV.R0 - SV.R0);
	des_SV.qd = (des_SV.q - SV.q)/d_time;
	SV = des_SV;
end

% Calculate links orientations, positions, velocities and accelerations
SV = calc_aa(LP,SV);
SV = calc_pos(LP,SV);
SV = calc_vel(LP,SV);
SV = calc_acc(LP,SV);

end