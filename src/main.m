clear, clc, close all
%% Connect to ROS Network
device = rosdevice('localhost');
 
if ~isCoreRunning(device) % run roslaunch ROSdistribution: Noetic
    bashConfig='source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash';
    bashLibraries = 'export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/noetic/lib"'
    bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    [status,cmdout] = system([bashConfig ';' bashLibraries ';' bashRunGazebo '&  echo $!'])
    % system([bashConfig ';' bashLibraries ';' bashRunGazebo])
    % system(['kill' cmdout]);   % end ros process 
    % system('killall -9 -v rosmaster')
    pause(7)
end

rosshutdown;
rosinit                 %('127.0.0.1',11311)

%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
bodyCam = rigidBody('camera');
addBody(robot,bodyCam,'gripper')
currentRobotJConfig = homeConfiguration(robot);

%% Initialize 
RoboCupManipulation_setInitialConfig;
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
ptCloudGlobal = pointCloud([0 0 0])  % pointcloud  enviroment estimation

%% Configure ROS nodes
% get joint_state
joint_state_sub = rossubscriber('/my_gen3/joint_states');
% send joint states commands
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoalMsg] = rosactionclient(ros_action);
% receive image data
ImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
% receive cloudpoint data
ptcSub = rossubscriber('/camera/depth/points','DataFormat','struct')
%% Algorith description
% sense ->  estimate enviroment -> segment -> identify->decide-> move -> repeat

%% sense
% get data from gazebo
curImage = receive(ImgSub);
img =readImage(ImgSub.LatestMessage);

xyz = rosReadXYZ(receive(ptcSub));

get_joint_msg = receive(joint_state_sub,1);
q_m = get_joint_msg.Position(2:8);
% sensed data: xyz, img, q_m

%% estimate enviroment
% pointcloud  enviroment estimation
[ptCloudSegment,MTH] = getPointCloud(xyz,img,robot,q_m);
gridStep = 0.01;
ptCloudGlobal = pcmerge(ptCloudGlobal,ptCloudSegment,gridStep);
%% segment 
distance = 0.05;
[labels, numClusters] = pcsegdist(ptCloudGlobal,distance);

%% identify 

%% decide 
% create collision mesh
xyzGlobal = ptCloudGlobal.Location;

m = collisionMesh(xyzGlobal);
ik = inverseKinematics('RigidBodyTree',robot);
pos_move=[0 0 -0.1] ;
rot_move= [0 0 -pi/10];
MTH_target= trvec2tform(pos_move)*eul2tform(rot_move)*MTH;
weights = [0.25 0.25 0.25 1 1 1];
initialguess = q_m';
[configSoln,solnInfo] = ik('camera',MTH_target,weights,initialguess);


target = configSoln';

%% Robot move
%RoboCupManipulation_setInitialConfig;

zeroVals = zeros(7,1);
q = target;
qd = zeroVals;
qdd = zeroVals;
t=rostime('now');
trajTimes = 2;
tolerance = [0.3 0.1 0.1];
msg = packageJointTrajectory(trajGoalMsg,q,qd,qdd,trajTimes,tolerance)
waitForServer(trajAct);
sendGoalAndWait(trajAct,msg)


% test gripper
pause(1)
action='close'

% action='open'
% gripper(action)
%% Plot
% plot point cloud

close all
pcshow(xyzGlobal,labels,'MarkerSize',10)
hold on
%show(m)

show(robot,q_m')
hold off
disp("Number of clusters " + numClusters)
% subplot(2,1,1)
% imshow(depth)
% subplot(2,1,2)
% imshow(img)