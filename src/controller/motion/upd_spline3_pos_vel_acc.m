%%%%%% Update
%%%%%% upd_spline3_pos_vel_acc
%%%%%% 
%%%%%% Calculate cubic spline parameters from two positions, initial velocity and final acceleration
%%%%%% 
%%%%%% Created 2020-01-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-14
%
%
% Calculate cubic spline curve coefficients for two points, given initial time, position and VELOCITY and the final time,
% position and ACCELERATION
%
%                              x(t) = a0 + a1*t + a2*t^2 + a3*t^3
%
%         a0..a3 : Cubic spline curve coefficients
%         x      : Position
%         t      : Time
%
% Given initial and final conditions, the coefficients are calculated by solving the following set of linear equations
%
%                              x0    = a0 + a1*t0 + a2*t0^2 + a3*t0^3
%                              xf    = a0 + a1*tf + a2*tf^2 + a3*tf^3
%                              x0_d  =    + a1    + 2*a2*t0 + 3*a3*t0^2
%                              xf_dd =    +       + 2*a2    + 6*a3*tf
%
%         x0     : Initial position
%         xf     : Final position
%         x0_d   : Initial velocity
%         xf_dd  : Final acceleration
%         t0     : Initial time
%         tf     : Final time
%
% Function variables:
%
%     OUTPUT
%         AA           : Cubic spline coefficients (4x1 vector)
%     INPUT
%         x0           : Initial position [m] (scalar)
%         xf           : Final position [m] (scalar)
%         x0_d         : Initial velocity [m/s] (scalar)
%         xf_dd        : Final acceleration [m/s^2] (scalar)
%         t0           : Initial time [s] (scalar)
%         tf           : Final time [s] (scalar)

function AA = upd_spline3_pos_vel_acc(x0, xf, x0_d, xf_dd, t0, tf)

XX = [x0; xf; x0_d; xf_dd ];

TT = [1   t0   t0^2   t0^3   ;
	  1   tf   tf^2   tf^3   ;
      0   1    2*t0   3*t0^2 ;
      0   0    2      6*tf   ];
          
AA = TT\XX;
        
        
end