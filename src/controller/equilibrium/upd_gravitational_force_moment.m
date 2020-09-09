%%%%%% Update
%%%%%% upd_gravitational_force_moment
%%%%%% 
%%%%%% Obtain Moment and Force due to gravity
%%%%%% 
%%%%%% Created 2020-04-23
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-23
%
%
% Obtain force and moment acting on the robot due to gravity acceleration at the CoM position
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         equilibrium_eval_param.Fg     : Force acting due to gravity acceleration [N] (3x1 vector)
%         equilibrium_eval_param.Mg     : Moment acting due to gravity acceleration [Nm] (3x1 vector)
%     INPUT
%         equilibrium_eval_param : Parameters for equilibrium evaluation (class)
%         LP           			 : Link Parameters (SpaceDyn class)
%         CoM          			 : Center of Mass position [m] (3x1 vector)


function equilibrium_eval_param = upd_gravitational_force_moment(equilibrium_eval_param, LP, CoM)

global Gravity

equilibrium_eval_param.Fg = LP.mass*Gravity;
equilibrium_eval_param.Mg = cross(CoM,LP.mass*Gravity);

end