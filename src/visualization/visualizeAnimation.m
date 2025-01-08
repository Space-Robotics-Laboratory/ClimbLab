% Visualization functions

animation.resetGraphicsObjects(time, robot, evaluation);

terrain.visualize(time);

robot.visualize();
robot.getKinematics().getReachableArea().visualize(robot, foothold_planning);

animation.setLight();

perception.visualize(robot);

trajectory_planning.visualize(time);

evaluation.visualize(robot, animation);

visualizeVectors();

animation.writeVideoFile();

% EOF
