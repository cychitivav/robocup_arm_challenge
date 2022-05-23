function moveto(robot, Tgoal, ROSobjects)

trajeAction = ROSobjects.trajeAction;
jntStateSub = ROSobjects.jntStateSub;

current_joint_state = jntStateSub.LatestMessage;
q = current_joint_state.Position(2:8);

[q, qd, qdd, trajeTimes] = computeTrajectory(q, Tgoal, robot, 'gripper', 1);
trajeAction.goalMsg = packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, [0 0.01 0.01]);

waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);
end
