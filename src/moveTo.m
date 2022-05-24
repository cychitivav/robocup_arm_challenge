function moveTo(robot, qCurrent,Tgoal, ROSobjects)

trajeAction = ROSobjects.trajeAction;
jntStateSub = ROSobjects.jntStateSub;

safeZoneHeight = 0.4;

% Up
qCurrent = receive(jntStateSub).Position(2:8);
Tup = getTransform(robot, qCurrent','gripper');
Tup(3,4) = safeZoneHeight;
[q, qd, qdd, trajeTimes] = computeTrajectory(qCurrent, Tup, robot, 'gripper', 1);
packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, [0 0 0]);


waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);

% Move
Tmove = Tgoal;
Tmove(3,4) = safeZoneHeight;
qCurrent = receive(jntStateSub).Position(2:8);
[q, qd, qdd, trajeTimes] = computeTrajectory(qCurrent, Tmove, robot, 'gripper', 1);
packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, [0 0 0]);

waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);

% Down
qCurrent = receive(jntStateSub).Position(2:8);
[q, qd, qdd, trajeTimes] = computeTrajectory(qCurrent, Tgoal, robot, 'gripper', 1);
packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, [0 0 0]);


waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);

end
