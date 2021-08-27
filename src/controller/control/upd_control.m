%%%%%% Update
%%%%%% upd_control
%%%%%% 
%%%%%% Calculate input variable
%%%%%% 
%%%%%% Created 2020-04-10
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-13
%
%
% Calculate intput variable
%
% Function variables:
%
%     OUTPUT
%         SV.tau        : Input torque [Nm] (nx1 vector)
%     INPUT
%         SV            : State Variables (SpaceDyn class)
%         des_SV        : Desired State Variables (SpaceDyn class)
%  		  control_param : Parameters for control (class)

function SV = upd_control(SV, des_SV, control_param)


switch control_param.type

case 'do_nothing'

case 'torque_PD'
	SV.tau = control_param.kp*(des_SV.q - SV.q) + control_param.kd*(des_SV.qd - SV.qd);

otherwise
	disp('Invalid control type');
end

end
