%% Connect to ROS Network
clear, clc, close all

device = rosdevice('localhost');                            %             
bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    
device = rosdevice('192.168.182.128','user','password');    % virtual machine

reset = false;
tic
if reset
    clear, clc, close all
    system(device,'killall -9 -v rosmaster')
    
else  
    rosshutdown;
    rosinit(device.DeviceAddress);
    %pause(5)
    setInitialConfig();
    
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

collisionObj = collisionCylinder(0.05,0.25);

for i = 1:robot.NumBodies
    if i < 7
        
        addCollision(robot.Bodies{i},collisionObj)
    end
end

%% Initialize 
%setInitialConfig();

physicsClient = rossvcclient('gazebo/unpause_physics');
call(physicsClient,'Timeout',3);
%% Configure ROS objects
% Create ROS subscriber to image, point cloud and joint states topics
ROSobjects.ImgSub = rossubscriber('/camera/color/image_raw');
ROSobjects.DepthSub = rossubscriber('/camera/depth/image_raw');

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

h = 0.4;

run disposeFixedObjectsTask.m

%% Algorithm description
% sense ->  estimate environment -> segment -> identify-> plot -> move -> repeat
controlPoints = [0.3 0 0.7]';

moveTo(robot,controlPoints, orient,ROSobjects)

pcRoi = [0 1 -0.5 0.5 -0.15 0.4];

[q,pcWorld,img,BW,depth] = sense(robot,ROSobjects,pcRoi);

% segment
distance = 0.05;
[labels, numClusters] = pcsegdist(pcWorld, distance);

posObj = [];

for k = 1:numClusters
        islabel = labels==k;
        xyzObj = pcWorld.Location(islabel,:);
        if(size(xyzObj,1)>500)
            posObj(k,:) =mean(xyzObj);
        end
end


run plotInfo.m

