clear, clc, close all
%% Connect to ROS Network
device = rosdevice('localhost');

runfromMATLAB = true;
 
if ~isCoreRunning(device) && runfromMATLAB % run roslaunch ROSdistribution: Noetic
    bashConfig='source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash';
    bashLibraries = 'export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/noetic/lib"';
    bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    [status,cmdout] = system([bashConfig ';' bashLibraries ';' bashRunGazebo '&  echo $!'])
    % system([ba    shConfig ';' bashLibraries ';' bashRunGazebo])
    % system(['kill' cmdout]);   % end ros process 
    % system('killall -9 -v rosmaster')
    pause(7)
    setInitialConfig();

end

rosshutdown;
rosinit                 %('127.0.0.1',11311)

%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');

% Add camera link to bracelet link
bodyCam = rigidBody('camera');
camera_MTH = trvec2tform([0 -0.055 -0.055])*eul2tform([0 pi 0],'xyz');
jnt = rigidBodyJoint('jnt','fixed');
setFixedTransform(jnt,camera_MTH)
bodyCam.Joint = jnt;

addBody(robot,bodyCam,'Bracelet_Link')

currentRobotJConfig = homeConfiguration(robot);

%% Initialize 
%setInitialConfig();

physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
%% Configure ROS objects
% Create ROS subscriber to image, point cloud and joint states topics
ROSobjects.ImgSub = rossubscriber('/camera/color/image_raw');
ROSobjects.jntStateSub = rossubscriber('/my_gen3/joint_states');
ROSobjects.ptcSub = rossubscriber('/camera/depth/points','DataFormat','struct');

% Create client and goal message to follow_joint_trajectory action 
ros_action_name = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory';
[trajeClient, trajeGoalMsg] = rosactionclient(ros_action_name);

ROSobjects.trajeAction.client = trajeClient;
ROSobjects.trajeAction.goalMsg = trajeGoalMsg;
%% Algorithm description
% sense ->  estimate environment -> segment -> identify-> plot -> move -> repeat
inspectionPoses = cat(3, ...
trvec2tform([0.4 -0.25 0.5]) * eul2tform(deg2rad([-160 10 0]), 'xyz'), ...
trvec2tform([0.4 0.25 0.5]) * eul2tform(deg2rad([160 10 0]), 'xyz'));

blueBinPose = trvec2tform([-0.1310 0.2910 0.7130]) * eul2tform([0 0 0],'xyz');
greenBinPose = trvec2tform([0 0 0]) * eul2tform([0 0 0],'xyz');
pcWorld = sense(robot,ROSobjects);
    %% sense 
    [pcCurrent,img,q] = sense(robot,ROSobjects);
    
    %% estimate enviroment
    pcWorld = updateWorld(robot, ROSobjects,pcCurrent,pcWorld)
    
    %% segment
    distance = 0.05;
    [labels, numClusters] = pcsegdist(pcWorld, distance);
    msh = collisionMesh(pcWorld.Location);
    %% identify 
     
    pos_move = [0 0 -0.1];
    rot_move = [0 0 -pi / 10];

    MTH_target = blueBinPose;
    %% plot
    run plotInfo.m
    
    %% move 
    moveto(robot, qCurrent,Tgoal, ROSobjects)

