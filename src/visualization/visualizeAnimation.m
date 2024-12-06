% Visualization functions

animation = animation.resetGraphicsObjects(time, robot);

terrain = terrain.visualize(time);

robot = robot.visualize(config_robot);

animation = animation.setLight();

motion_planning = motion_planning.visualize(time);

visualizeVectors();

drawnow limitrate nocallbacks;
writeVideo(animation.getSimulationVideo(), getframe(animation.getFigure()));

% EOF