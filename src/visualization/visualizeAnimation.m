% Visualization functions

animation.resetGraphicsObjects(time, robot);

terrain.visualize(time);

robot.visualize();

animation.setLight();

trajectory_planning.visualize(time);

visualizeVectors();

drawnow limitrate nocallbacks;
writeVideo(animation.getSimulationVideo(), getframe(animation.getFigure()));

% EOF
