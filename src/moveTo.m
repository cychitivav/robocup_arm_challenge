function moveTo(robot,controlPoints, orient,ROSobjects)


trajeAction = ROSobjects.trajeAction;
jntStateSub = ROSobjects.jntStateSub;
endEffector = 'gripper';

qCurrent = receive(jntStateSub).Position(2:8)';
MTH= getTransform(robot, qCurrent, endEffector);

posCurrent = MTH(1:3,4);

controlPoints = [posCurrent, controlPoints];

[q, qd, qdd, trajeTimes, dt] = computeTraj(qCurrent, controlPoints,orient, robot,endEffector);


tolerance = [ max(q,[],'all'), max(qd,[],'all'),max(qdd,[],'all')];
tolerance = tolerance.*[0.2 0.2 0.5];

packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, tolerance);


disp("trajectory ended")
pause(1)
waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);
clc

disp("trajectory ended")
end
