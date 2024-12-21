% Create variables to be saved

variable_names = who;

if (time == 0.0)
  cnt = uint8(1);
elseif (save_settings.variable_saving_time_interval >= world.time_step)
  if (rem(time, save_settings.variable_saving_time_interval) > eps)
    return;
  end
  cnt = uint8(numel(variables_log.time) + 1);
else
  error("ERROR: variable saving time interval should be larger than time-step!");
end


variables_log.time(cnt, 1) = time;
variables_log.base_position(cnt, :) = robot.SV_.getBasePosition()';
variables_log.base_orientation(cnt, :) = robot.SV_.getBaseOrientationEuler()';
variables_log.base_linear_velocity(cnt, :) = robot.SV_.getBaseLinearVelocity()';
variables_log.base_angular_velocity(cnt, :) = robot.SV_.getBaseAngularVelocity()';
variables_log.base_linear_acceleration(cnt, :) = robot.SV_.getBaseLinearAcceleration()';
variables_log.base_angular_acceleration(cnt, :) = robot.SV_.getBaseAngularAcceleration()';
variables_log.joint_angular_position(cnt, :) = robot.SV_.getJointAngularPosition()';
variables_log.joint_angular_velocity(cnt, :) = robot.SV_.getJointAngularVelocity()';
variables_log.joint_angular_acceleration(cnt, :) = robot.SV_.getJointAngularAcceleration()';
variables_log.joint_torque(cnt, :) = robot.SV_.getJointTorque()';

Fe = robot.SV_.getGroundReactionForce(robot.LP_);
Te = robot.SV_.getGroundReactionMoment(robot.LP_);
EE_positions = robot.getEEPosition();
for limb_id = 1 : robot.LP_.getNumberOfLimb()
  variables_log.(matlab.lang.makeValidName("GRF_" + num2str(limb_id)))(cnt, :) = Fe(:, limb_id)';
  variables_log.(matlab.lang.makeValidName("GRF_norm_" + num2str(limb_id)))(cnt, :) = ...
    norm(Fe(:, limb_id));
  variables_log.(matlab.lang.makeValidName("GRM_" + num2str(limb_id)))(cnt, :) = Te(:, limb_id)';
  variables_log.(matlab.lang.makeValidName("GRM_norm_" + num2str(limb_id)))(cnt, :) = ...
    norm(Te(:, limb_id));

  variables_log.(matlab.lang.makeValidName("EE_position_" + num2str(limb_id)))(cnt, :) = ...
    EE_positions(:, limb_id)';
end


if(save_settings.getSaveManipulability())
  variables_log.manipulability_measure(cnt, :) = evaluation.getManipulability().getManipulabilityMeasure();
end

if (save_settings.getSaveDynamicManipulability())
  variables_log.dynamic_manipulability_measure(cnt, :) = evaluation.getManipulability().getDynamicManipulabilityMeasure();
end

if (save_settings.getSaveTumbleStabilityMargin())
  variables_log.TSM(cnt, 1) = evaluation.getTumbleStabilityMargin().getTumbleStabilityMargin();
end

clearvars('-except', variable_names{:});
% EOF
