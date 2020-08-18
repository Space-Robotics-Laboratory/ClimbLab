%%%%%% Update
%%%%%% upd_inertial_force_moment_plus_rotational
%%%%%% 
%%%%%% Obtain Moment and Force due to inertial acceleraion
%%%%%% 
%%%%%% Created 2020-04-23
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-23
%
%
% Obtain force and moment acting on the robot due to inertial acceleration, including angular
% velocity and acceleration effects of each link
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.Fa     : Force acting due to inertial acceleration [N] (3x1 vector)
%         equilibrium_eval_param.Ma     : Moment acting due to inertial acceleration [Nm] (3x1 vector)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         LP           			 : Link Parameters (SpaceDyn class)
%         SV           			 : State Variables (SpaceDyn class)


function equilibrium_eval_param = upd_inertial_force_moment_plus_rotational(equilibrium_eval_param, LP, SV)

% Force due to each link linear acceleration
equilibrium_eval_param.Fa = LP.m0*SV.vd0;
for i=1:LP.num_q
    equilibrium_eval_param.Fa = equilibrium_eval_param.Fa + LP.m(i)*SV.vd(:,i);
end

% Moment due to each link linear acceleration
equilibrium_eval_param.Ma = cross(LP.m0*SV.R0,SV.vd0);
for i=1:LP.num_q
    equilibrium_eval_param.Ma = equilibrium_eval_param.Ma + cross(LP.m(i)*SV.RR(:,i),SV.vd(:,i));
end
% Moment due to each link angular acceleration
equilibrium_eval_param.Ma = equilibrium_eval_param.Ma + (LP.inertia0*SV.wd0 + cross(SV.w0, LP.inertia0*SV.w0) );
for i=1:LP.num_q
    equilibrium_eval_param.Ma = equilibrium_eval_param.Ma + (LP.inertia(:,3*i-2:3*i)*SV.wd(:,i) + ...
    	                        cross( SV.ww(:,i), LP.inertia(:,3*i-2:3*i)*SV.ww(:,i) ) );
end


end