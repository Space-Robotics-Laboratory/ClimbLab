% Update functions

disp(time);

% Foothold Planning
foothold_planning.plan(time, terrain, path_planning, gait_planning);

% Gait Planning
gait_planning.plan(time, robot, path_planning, foothold_planning);

% Trajectory Planning
trajectory_planning.plan(time, robot, foothold_planning, gait_planning);

% Limb Controller
robot = limb_controller.control(time, robot, foothold_planning, gait_planning, trajectory_planning, ...
  terrain);

% Joint Controller
robot = joint_controller.control(robot);

% Interaction between robot and environment
robot.updateGripperState(terrain);
robot.calcGroundReactionForces(terrain);

% Forward Dynamics
robot = robot.forwardDynamics();

% Forward Kinematics
robot.forwardKinematics();

% Evaluation
robot.SV_.calcCoM(robot.getLinkParameter());
evaluation.evaluate(robot);

% Save variables
saveVariables();

% EOF
