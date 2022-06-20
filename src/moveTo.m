function moveTo(robot,controlPoints, orient,ROSobjects)
factor=2;

trajeAction = ROSobjects.trajeAction;
jntStateSub = ROSobjects.jntStateSub;
endEffector = 'gripper';

qCurrent = receive(jntStateSub).Position(2:8)';
MTH= getTransform(robot, qCurrent, endEffector);

posCurrent = MTH(1:3,4);

controlPoints = [posCurrent, controlPoints];

[q, qd, qdd, trajeTimes, dt] = computeTraj(qCurrent, controlPoints,orient, robot);
trajeTimes = trajeTimes/factor;
dt= dt/factor;

tolerance = [ max(q,[],'all'), max(qd,[],'all'),max(qdd,[],'all')];
tolerance = tolerance.*[0.01 0.01 0.1];

packageJointTrajectory(trajeAction.goalMsg, q, qd, qdd, trajeTimes, tolerance);


waitForServer(trajeAction.client);
sendGoalAndWait(trajeAction.client, trajeAction.goalMsg);

end
