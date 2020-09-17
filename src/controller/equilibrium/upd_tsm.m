%%%%%% Update
%%%%%% upd_tsm
%%%%%% 
%%%%%% Calculate TSM
%%%%%% 
%%%%%% Created 2020-05-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-08
%
%
% Calculate Tumble Stability Margin
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.tsm                   : Tumble Stability Margin [m] (scalar)
%         equilibrium_eval_param.tsm_equilibrium_flag  : Flag for equilibrium based on TSM (scalar)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         LP           			 : Link Parameters (SpaceDyn class)


function equilibrium_eval_param = upd_tsm(equilibrium_eval_param, LP)

global Gravity

Mab = equilibrium_eval_param.Mab;
% If a tumbling axis was judged as tumbling, tumbling moment Mab is zero
Mab(equilibrium_eval_param.tsm_axes_judgment == 0) = 0;

% Tumble Stability Margin (TSM) [m]
equilibrium_eval_param.tsm = min(abs(Mab))/(abs(LP.mass*norm(Gravity)));
% Equilibrium flag
if equilibrium_eval_param.tsm == 0
	equilibrium_eval_param.tsm_equilibrium_flag = 0;
else
	equilibrium_eval_param.tsm_equilibrium_flag = 1;
end

end