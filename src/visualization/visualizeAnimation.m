% Visualization functions

animation = animation.resetGraphicsObjects(time, robot);

terrain = terrain.visualize(time);

robot = robot.visualize();

animation = animation.setLight();

trajectory_planning = trajectory_planning.visualize(time);

visualizeVectors();

drawnow limitrate nocallbacks;
writeVideo(animation.getSimulationVideo(), getframe(animation.getFigure()));

% EOF
