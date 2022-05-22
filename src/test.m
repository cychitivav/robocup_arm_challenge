clc, clear, close all
%% Initialize 
rosshutdown
rosinit

setInitialConfig;
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');

jointSub = rossubscriber('/my_gen3/joint_states');
jointMsg = receive(jointSub,2);
currentRobotJConfig =  jointMsg.Position(2:8);
%% TFtree
tree = rostf('DataFormat','struct');
pause(1);
tree.AvailableFrames;
%% First inspection pose
Tend = trvec2tform([0.4 0.25 0.5])*eul2tform(deg2rad([160 10 0]),'xyz');

[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');
[q,qd,qdd,trajTimes] = computeTrajectory(currentRobotJConfig, Tend, robot, 'gripper', 4);

trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
waitForServer(trajAct);
sendGoalAndWait(trajAct,trajGoal);
% Plot
jointMsg = receive(jointSub,2);
currentRobotJConfig =  jointMsg.Position(2:8);
show(robot, currentRobotJConfig')
axis(0.6*[-0.2 1 -1 1 -0.2 1.5])
%% Save point cloud
tf = getTransform(tree,'world','camera_link','Timeout',5);

tform = trvec2tform([tf.Transform.Translation.X, ...
                     tf.Transform.Translation.Y, ...
                     tf.Transform.Translation.Z]) * ...
        quat2tform([tf.Transform.Rotation.X, ...
                    tf.Transform.Rotation.Y, ...
                    tf.Transform.Rotation.Z, ...
                    tf.Transform.Rotation.W]);

pcSub = rossubscriber('/camera/depth/points');
pc = receive(pcSub,2);
pcobj = pointCloud(readXYZ(pc),'Color',uint8(255*readRGB(pc))); 

pcWorld = pctransform(pcobj,rigid3d(tform'));
%% Second inspection pose
Tend = trvec2tform([0.4 -0.25 0.5])*eul2tform(deg2rad([-160 10 0]),'xyz');

[trajAct,trajGoal] = rosactionclient( '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');
[q,qd,qdd,trajTimes] = computeTrajectory(currentRobotJConfig, Tend, robot, 'gripper', 4);

trajGoal = packageJointTrajectory(trajGoal,q,qd,qdd,trajTimes);
waitForServer(trajAct);
sendGoalAndWait(trajAct,trajGoal);
% Plot
jointMsg = receive(jointSub,2);
currentRobotJConfig =  jointMsg.Position(2:8);
show(robot, currentRobotJConfig')
axis(0.6*[-0.2 1 -1 1 -0.2 1.5])
%% Save point cloud
tf = getTransform(tree,'world','camera_link','Timeout',5);

tform = trvec2tform([tf.Transform.Translation.X, ...
                     tf.Transform.Translation.Y, ...
                     tf.Transform.Translation.Z]) * ...
        quat2tform([tf.Transform.Rotation.X, ...
                    tf.Transform.Rotation.Y, ...
                    tf.Transform.Rotation.Z, ...
                    tf.Transform.Rotation.W]);

pcSub = rossubscriber('/camera/depth/points');
pc = receive(pcSub,2);
pcobj = pointCloud(readXYZ(pc));

pcWorld2 = pctransform(pcobj,rigid3d(tform'));
% MSH = collisionMesh(double(pcWorld.Location));


pcEnd = pcmerge(pcWorld, pcWorld2, 0.1);
hold on
pcshow(pcWorld)
axis tight

tform2rotm
