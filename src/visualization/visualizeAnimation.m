% Visualization functions

animation.resetGraphicsObjects(time, robot);

terrain.visualize(time);

robot.visualize();

animation.setLight();

trajectory_planning.visualize(time);

evaluation.visualize();

visualizeVectors();

animation.writeVideoFile();

% EOF
