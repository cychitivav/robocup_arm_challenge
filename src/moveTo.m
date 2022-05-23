function moveTo(robot, qCurrent,Tgoal, ROSobjects)

trajeAction = ROSobjects.trajeAction;


[q, qd, qdd, trajeTimes] = computeTrajectory(qCurrent, Tgoal, robot, 'gripper', 1);
packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, [0 0 0]);


waitForServer(trajeAction.client);
[resultMsg,state,status] = sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);

end
