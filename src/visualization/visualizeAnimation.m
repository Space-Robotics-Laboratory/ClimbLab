% Visualization functions

animation.resetGraphicsObjects(time, robot, evaluation);

terrain.visualize(time);

robot.visualize();

animation.setLight();

trajectory_planning.visualize(time);

evaluation.visualize(terrain, robot, animation);

visualizeVectors();

animation.writeVideoFile();

% EOF
