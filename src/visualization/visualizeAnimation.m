% Visualization functions

animation.resetGraphicsObjects(time, robot, evaluation);

terrain.visualize(time);

robot.visualize();

animation.setLight();

perception.visualize();

trajectory_planning.visualize(time);

evaluation.visualize(robot, animation);

visualizeVectors();

animation.writeVideoFile();

% EOF
