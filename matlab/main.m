clear, clc, close all
%% Connect to ROS Network
rosshutdown
rosinit('localhost',11311)
%% Load robot model and set initial config
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
currentRobotJConfig = homeConfiguration(robot);

RoboCupManipulation_setInitialConfig;
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
%% Configure ROS subscriber 
joint_state_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoalMsg] = rosactionclient(ros_action);

rgbImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
rgbDptSub = rossubscriber('/camera/depth/image_raw');     % depth sensor
%% Test image processing

curImage = receive(rgbImgSub);
img = readImage(rgbImgSub.LatestMessage);

curDepth = receive(rgbDptSub);
depth = readImage(rgbDptSub.LatestMessage); 
%depth=uint8(depth*255);

%% Plot
close all
subplot(2,1,1)
imshow(depth)
subplot(2,1,2)
imshow(img)