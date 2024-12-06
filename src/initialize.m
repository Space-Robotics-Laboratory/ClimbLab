% Initialization functions

% Configuration
config_world = ConfigWorld(config);
config_terrain = ConfigTerrain(config);
config_robot = ConfigRobot(config);
config_path_planning = ConfigPathPlanning(config);
config_foothold_planning = ConfigFootholdPlanning(config);
config_gait_planning = ConfigGaitPlanning(config);
config_motion_planning = ConfigMotionPlanning(config);
config_joint_controller = ConfigJointController(config);
config_animation_settings = ConfigAnimationSettings(config);
save_settings = ConfigSaveSettings(config, config_world);

% Animation
animation = Animation(config_animation_settings);
animation = animation.createVideoFile(run_cod, run_id, run_date);

% Environment
world = World(config_world);
max_sim_time = world.getMaxSimulationTime();
terrain = Terrain(config_terrain);
terrain = terrain.initialize();

% Robot
robot = Robot(config_robot, world, terrain);

% Path Planning
path_planning = PathPlanning(config_path_planning, robot, terrain);
path_planning = path_planning.plan(robot);

% Foothold Planning
foothold_planning = FootholdPlanning(config_foothold_planning, robot);

% Gait Planning
gait_planning = GaitPlanning(config_gait_planning);

% Trajectory Planning
motion_planning = MotionPlanning(config_motion_planning, robot);

% Limb Controller
limb_controller = LimbController();

% Joint Controller
joint_controller = JointController(config_joint_controller);


variables_log = [];

% EOF