
if (animation.getGroundReactionForceVectorShow())
  animation = robot.visualizeForceVectors(animation);
end

evaluation.getGIA().visualizeGIAVector(robot, animation);
