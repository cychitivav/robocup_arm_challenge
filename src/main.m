%% Connect to ROS Network
clear, clc, close all

device = rosdevice('localhost');                            %             

device = rosdevice('192.168.182.128','user','password');    % virtual machine

reset = false;
tic
if reset
    clear, clc, close all
    system(device,'killall -9 -v rosmaster')
    
else  
    rosshutdown;
    rosinit(device.DeviceAddress);
    pause(3)
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

%% Algorithm description
% sense ->  estimate environment -> segment -> identify-> plot -> move -> repeat

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

%% move
orient = [pi 0 pi/2];

inspection = [0.4 -0.25  0.5
              0.4  0.25  0.5];
h=0.7;

posBotlBin = [-0.3 -0.45 h];        % blue bin
posCanBin = [-0.3 0.45 h];          % greeBin

controlPoints = inspection';

posTarget = [0.0285    0.4730    0.0900];
posBin = posBotlBin;

controlPoints = [posTarget;posBin]';

%controlPoints = [0.3 0 0.7]';



moveTo(robot,controlPoints, orient,ROSobjects)

%activateGripper('open',ROSobjects);




%% save data
pos = [0 0 0];
orient = [0 0 0];
trainData= table();
posObj = [];

envData= table();
envData(end+1,:) = {pos,orient,q,img, depth, pcWorld, numClusters};

lab=["f","b","l","b","l"];
msh={};

for k = 1:numClusters
        
        islabel = labels==k;
        xyzObj = pcWorld.Location(islabel,:);
        row = {pointCloud(xyzObj), mean(xyzObj),size(xyzObj,1),pca(zscore(xyzObj))};
        trainData(k,:) = row;
        msh{k} = collisionMesh(xyzObj);
        %trainData(k,2) = {lab(k)};
end



save('data.mat','robot','pcWorld','q','img','posObj','controlPoints','depth','BW','labels','trainData','envData')


%%
pos = [ 0.0285    0.4730    0.0900
       -0.0430    0.6161   -0.0800
        0.3540    0.5508   -0.0146
        0.3       0.4       0
        0.4       0.2       0
        0.35      0         0   
        0.35      -0.2      0
        0.3       -0.4      0];

  
%%
    Z = zscore(pcWorld.Location); % Standardized data
    [general,score] = pca(Z);
   

%run disposeFixedObjectsTask.m
%%
% numPoints = pcWorld.Count;
% pointsDim = 3;
% numClasses = 3;
% 
% lgraph = pointnetplusLayers(numPoints,pointsDim,numClasses, ...
%     NormalizationLayer="instance", ...
%     NumSetAbstractionModules=3, ...
%     NumClusters=2048, ...
%     ClusterRadius=0.1, ...
%     ClusterSize=32, ...
%     PointNetLayerSize=32);
%analyzeNetwork(lgraph)

%%
% inputChannelSize = 3;
% hiddenChannelSize1 = [64,128];
% hiddenChannelSize2 = 256;
% [parameters.InputTransform, state.InputTransform] = initializeTransform(inputChannelSize,hiddenChannelSize1,hiddenChannelSize2);
