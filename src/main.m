clear, clc, close all
%% Connect to ROS Network

device = rosdevice('localhost');
if ~isCoreRunning(device)
    shellLibraries = 'export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/noetic/lib"';
    shellRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    [status,cmdout] = system([shellLibraries ';' shellRunGazebo '&  echo $!'])
    % system(['kill' cmdout]);   % end ros process 
    % system('killall -9 rosmaster')
    pause(7)
end

rosshutdown;
rosinit %('127.0.0.1',11311)

%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');

currentRobotJConfig = homeConfiguration(robot);

%% Initialize 
RoboCupManipulation_setInitialConfig;
physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);

%% Configure ROS subscribers
joint_state_sub = rossubscriber('/my_gen3/joint_states');
ros_action = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajAct,trajGoalMsg] = rosactionclient(ros_action);

ImgSub = rossubscriber('/camera/color/image_raw');     % camera sensor
DptSub = rossubscriber('/camera/depth/image_raw');     % depth sensor
topic='/camera/depth/points'
sub= rossubscriber(topic)
%% Test image processing

curImage = receive(ImgSub);
img = readImage(ImgSub.LatestMessage);

curDepth = receive(DptSub);
depth = readImage(DptSub.LatestMessage); 
%depth=uint8(depth*255);
%%
pause(1)
action='close'

% action='open'
% gripper(action)
%% Plot
close all
subplot(2,1,1)
imshow(depth)
subplot(2,1,2)
imshow(img)