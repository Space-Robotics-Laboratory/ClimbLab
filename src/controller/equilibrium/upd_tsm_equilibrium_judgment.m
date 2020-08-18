%%%%%% Update
%%%%%% upd_tsm_equilibrium_judgment
%%%%%% 
%%%%%% Calculate tumbling moment for axes
%%%%%% 
%%%%%% Created 2020-05-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-08
%
%
% Judgment of equilibrium based on tumbling moment
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.tumbling_axes         : Number of legs that makes a tumbling axis (tumbling_axes_numberx2 matrix)
%         equilibrium_eval_param.tumbling_axes_number  : Total number of tumbling axes (scalar)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)

function equilibrium_eval_param = upd_tsm_equilibrium_judgment(equilibrium_eval_param, LP, SV, POS_e)

equilibrium_eval_param.tsm_axes_judgment = zeros(1,equilibrium_eval_param.tumbling_axes_number);

for i = 1:equilibrium_eval_param.tumbling_axes_number
	% Current tumbling axis legs' number
	a = equilibrium_eval_param.tumbling_axes(i,1);
	b = equilibrium_eval_param.tumbling_axes(i,2);
	% Current tumbling axis legs' positions
	pa = POS_e(:,a);
	pb = POS_e(:,b);
	for j = 1:LP.num_limb
		if j ~= a && j ~= b && SV.sup(j) == 1
			pj = POS_e(:,j);
			% Obtain surface normal for pj
			nj = [0 0 1]';

			not_tumbling_condition = (cross((pj - pa),nj))'*(equilibrium_eval_param.Mab(i)*(pa - pb)/abs(norm(pa - pb)));
			if not_tumbling_condition > 0
				equilibrium_eval_param.tsm_axes_judgment(i) = 1;
				break
			end
		end
	end
end


end