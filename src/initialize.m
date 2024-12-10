% Initialization functions

% Configuration
config_world = ConfigWorld(config);
config_terrain = ConfigTerrain(config);
config_robot = ConfigRobot(config);
config_path_planning = ConfigPathPlanning(config);
config_foothold_planning = ConfigFootholdPlanning(config);
config_gait_planning = ConfigGaitPlanning(config);
config_trajectory_planning = ConfigTrajectoryPlanning(config);
config_joint_controller = ConfigJointController(config);
config_animation_settings = ConfigAnimationSettings(config);
save_settings = ConfigSaveSettings(config, config_world);

% Animation
animation = Animation(config_animation_settings);
animation.createVideoFile(run_cod, run_id, run_date);

% Environment
world = World(config_world);
kMaxSimTime = world.getMaxSimulationTime();
terrain = Terrain(config_terrain);

% Robot
robot = Robot(config_robot, world, terrain);

% Path Planning
path_planning = PathPlanning(config_path_planning, robot, terrain);
path_planning.plan(robot);

% Foothold Planning
foothold_planning = FootholdPlanning(config_foothold_planning, robot);

% Gait Planning
gait_planning = GaitPlanning(config_gait_planning);

% Trajectory Planning
trajectory_planning = TrajectoryPlanning(config_trajectory_planning, robot);

% Limb Controller
limb_controller = LimbController();

% Joint Controller
joint_controller = JointController(config_joint_controller);


variables_log = [];

% EOF
