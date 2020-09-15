%%%%%% Update
%%%%%% upd_tsm_tumbling_moment
%%%%%% 
%%%%%% Calculate tumbling moment for axes
%%%%%% 
%%%%%% Created 2020-05-06
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-06
%
%
% Calculate tumbling moment
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.Mab         : Tumbling moment for each tumbling axis (1xtumbling_axes_number vector)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         LP           			 : Link Parameters (SpaceDyn class)
%         SV           			 : State Variables (SpaceDyn class)
%         POS_e                  : Position of the end-effector [m] (3xnum_limb matrix)
%         F_grip                 : Maximum gripping force [N] (scalar)

function equilibrium_eval_param = upd_tsm_tumbling_moment(equilibrium_eval_param, LP, SV, POS_e, F_grip)

% Initialize tumbling moment Mab 
equilibrium_eval_param.Mab = zeros(1,equilibrium_eval_param.tumbling_axes_number);

for i = 1:equilibrium_eval_param.tumbling_axes_number
	% Current tumbling axis legs' number
	a = equilibrium_eval_param.tumbling_axes(i,1);
	b = equilibrium_eval_param.tumbling_axes(i,2);
	% Current tumbling axis legs' positions
	pa = POS_e(:,a);
	pb = POS_e(:,b);

	% Moment due to inertial and gravitational forces around tumbling axis
	Mb = equilibrium_eval_param.Ma - equilibrium_eval_param.Mg;
	Fb = equilibrium_eval_param.Fa - equilibrium_eval_param.Fg;
	equilibrium_eval_param.Mab(i) = Mb'*(pa-pb)/abs(norm(pa-pb)) + Fb'*(cross(pb,pa))/abs(norm(pa-pb));

	% If there is a gripping force
	if F_grip ~= 0
		% Check all possible gripping points besides the ones forming the tumbling axis
		for j = 1:LP.num_limb
			if j ~= a && j ~= b && SV.sup(j) == 1
				pj = POS_e(:,j);
				% Obtain surface normal for pj
				nj = [0 0 1]';

				% Gripping force direction
				ng = (cross((pa-pj),(pb-pj)))/abs(norm(cross((pa-pj),(pb-pj))));

				if ng'*nj > 0
					sgnx = -1;
				elseif ng'*nj < 0
					sgnx = 1;
				else
					sgnx = 0;
				end
				Fgripper = sgnx * F_grip * ng;

				% Update tumbling moment with gripping force
				equilibrium_eval_param.Mab(i) = equilibrium_eval_param.Mab(i) - Fgripper'*(cross((pb-pj),(pa-pj))/abs(norm(pa-pb))); 
			end
		end
	end
end

end