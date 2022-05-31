clear, clc, close all
%% Connect to ROS Network
device = rosdevice('localhost');                            %             

device = rosdevice('192.168.182.130','user','password');    % virtual machine

runfromMATLAB = true;
tic
if ~isCoreRunning(device) && runfromMATLAB % run roslaunch ROSdistribution: Noetic
%     bashConfig=['source /opt/ros/' ROS_DISTRO '/setup.bash; source ~/catkin_ws/devel/setup.bash'];
%     bashLibraries = ['export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/' ROS_DISTRO  '/lib"'];
%     bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
%     %[status,cmdout] = 
%     system(device,[bashConfig ';' bashLibraries ';' bashRunGazebo '&  echo $!; sleep 15'])
    
    bashROSinfo= 'export | grep ROS';
    bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    
    command = ['printf " export DISPLAY=:0 \n' bashRunGazebo '>dump.txt& \n sleep 10 \n jobs -l \n disown #1  " > remotelaunch.bash; bash -i remotelaunch.bash  ; jobs -l '];
    

    system(device,command)
    % kill ros process
    % system(['kill' cmdout]);   % end ros process 
    % system(device,'killall -9 -v rosmaster')
    
    pause(7)
    system(device,'cat dump.txt')
    % system(['kill' cmdout]);   % end ros process 
    % system(device,'killall -9 -v rosmaster')
    pause(7)
    rosshutdown;
    rosinit(device.DeviceAddress)
    setInitialConfig();

else  
    rosshutdown;
    rosinit(device.DeviceAddress);
end
toc

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

% Create client and goal message to gripper action 
ros_action_name = '/my_gen3/custom_gripper_controller/gripper_cmd';
[gripClient,gripGoalMsg] = rosactionclient(ros_action_name);

ROSobjects.gripAction.client = gripClient;
ROSobjects.gripAction.goalMsg = gripGoalMsg;

%% Algorithm description
% sense ->  estimate environment -> segment -> identify-> plot -> move -> repeat
inspectionPoses = cat(3, ...
    trvec2tform([0.4 -0.25 0.5]) * eul2tform(deg2rad([-160 10 0]), 'xyz'), ...
    trvec2tform([0.4 0.25 0.5]) * eul2tform(deg2rad([160 10 0]), 'xyz'));

h=0.3;
orient=[pi 0 0];
blueBinPose = trvec2tform([-0.3 -0.45 h]) * eul2tform(orient,'xyz');
greenBinPose = trvec2tform([-0.3 0.45 h]) * eul2tform(orient,'xyz');

zoneLeft = trvec2tform([0.3 0.45 h]) * eul2tform(orient,'xyz');
zoneRight = trvec2tform([0.4 0.3 h]) * eul2tform(orient,'xyz');
zoneMiddle = trvec2tform([0.4 0 h]) * eul2tform(orient,'xyz');

pcRoi = [0 1 -0.5 0.5 -0.15 0.3];
pcWorld = sense(robot,ROSobjects,pcRoi);

activateGripper('open',ROSobjects);

%run disposeFixedObjectsTask.m

waypoints ={zoneLeft,greenBinPose,zoneLeft,zoneMiddle,zoneRight};
for k=1:size(waypoints,2)

   
    %% sense 
    [pcCurrent,img,q] = sense(robot,ROSobjects,pcRoi);
    
    %% estimate enviroment
    pcWorld = updateWorld(robot, ROSobjects,pcCurrent,pcWorld);
    
    %% segment
    distance = 0.05;
    [labels, numClusters] = pcsegdist(pcWorld, distance);
    msh = collisionMesh(pcWorld.Location);
    %% identify 
     
    pos_move = [0 0 -0.1];
    rot_move = [0 0 -pi / 10];

    MTH_target = waypoints{k};
    %% plot
    run plotInfo.m
    
    %% move 
    moveTo(robot, q,MTH_target, ROSobjects)
    
    % display info
    disp("Number of clusters " + numClusters)
end
