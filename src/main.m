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
ptCloudGlobal = pointCloud([0 0 0]);  % pointcloud  enviroment estimation

%% Configure ROS nodes
% get joint_state
ROSNodes.joint_state_sub = rossubscriber('/my_gen3/joint_states');
% send joint states commands
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[ROSNodes.trajAct, trajGoalMsg] = rosactionclient(ros_action);
% receive image data
ROSNodes.ImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
% receive cloudpoint data
ROSNodes.ptcSub = rossubscriber('/camera/depth/points','DataFormat','struct');
 
%% Algorith description
% sense ->  estimate enviroment -> segment -> identify-> decide-> move -> repeat
tic
[msg,xyzGlobal,labels,numClusters, q_m,MTH_target] = algorithm(ROSNodes,trajGoalMsg,robot,ptCloudGlobal);
time_spent=toc;
sendGoalAndWait(ROSNodes.trajAct,msg)


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