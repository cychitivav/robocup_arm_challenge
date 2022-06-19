%% Connect to ROS Network
clear, clc, close all

device = rosdevice('localhost');                            %             

device = rosdevice('192.168.182.130','user','password');    % virtual machine

runfromMATLAB = false;
tic
if ~isCoreRunning(device) && runfromMATLAB 
    
    bashROSinfo= 'export | grep ROS';
    bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    bashLibraries = 'export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/noetic/lib"';
    
    createBash = ['printf " \n' bashRunGazebo '>dump.txt& \n sleep 10 \n jobs -l \n disown #1  " > remotelaunch.bash;']
    runBash=[bashLibraries '; bash -i remotelaunch.bash  ; jobs -l '];
    
    system(device,createBash)

    system(device,runBash)

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
inspectionPoses = cat(3, ...
    trvec2tform([0.4 -0.25 0.5]) * eul2tform(deg2rad([-160 10 0]), 'xyz'), ...
    trvec2tform([0.4 0.25 0.5]) * eul2tform(deg2rad([160 10 0]), 'xyz'));

h=0.3;
orient=[pi 0 0];

pcRoi = [0 1 -0.5 0.5 -0.15 0.4];
[pcWorld,img,q] = sense(robot,ROSobjects,pcRoi);
depth = readImage(receive(ROSobjects.DepthSub));
BW = segmentImage(img);


% segment
distance = 0.05;
[labels, numClusters] = pcsegdist(pcWorld, distance);
    


% save data
pos = [0 0 0];
orient = [0 0 0];
trainData= table();
posObj = [];

envData= table();
envData(end+1,:) = {pos,orient,q,img, depth, pcWorld, numClusters};

lab=["f","b","l","b","l"];

for k = 1:numClusters
        
        islabel = labels==k;
        xyzObj = pcWorld.Location(islabel,:);
        trainData(k,1) = {pointCloud(xyzObj)};
        trainData(k,2) = {mean(xyzObj)};
        posObj(k,:) =mean(xyzObj);
        
        %trainData(k,2) = {lab(k)};
end



save('data.mat','robot','pcWorld','q','img','posObj','depth','BW','trainData','envData')


run plotInfo.m

%activateGripper('open',ROSobjects);

%%
    xyzWorld=pcWorld.Location;
    figure()
    pcshow(xyzWorld,labels)
    hold on
    scatter3(posObj(:,1),posObj(:,2),posObj(:,3),'filled','SizeData',100)
    colormap(hsv(numClusters))
    colorbar
    hold off
%%
    Z = zscore(xyzWorld); % Standardized data
    [general,score] = pca(Z);
   
    
    msh={};

figure()
plot3(general(:,1),general(:,2),general(:,3))
hold on
    for k = 1:numClusters
        islabel = labels==k;
        local = pca(xyzWorld(islabel,:));
        msh{k} = collisionMesh(xyzWorld(islabel,:));
        plot3(local(:,1),local(:,2),local(:,3))
    end     
legend


%run disposeFixedObjectsTask.m
%%
numPoints = pcWorld.Count;
pointsDim = 3;
numClasses = 3;

lgraph = pointnetplusLayers(numPoints,pointsDim,numClasses, ...
    NormalizationLayer="instance", ...
    NumSetAbstractionModules=3, ...
    NumClusters=2048, ...
    ClusterRadius=0.1, ...
    ClusterSize=32, ...
    PointNetLayerSize=32);
%analyzeNetwork(lgraph)

%%
% inputChannelSize = 3;
% hiddenChannelSize1 = [64,128];
% hiddenChannelSize2 = 256;
% [parameters.InputTransform, state.InputTransform] = initializeTransform(inputChannelSize,hiddenChannelSize1,hiddenChannelSize2);
%% Task 
        % waypoints ={zoneLeft,greenBinPose,zoneLeft,zoneMiddle,zoneRight};
% 
% for k=1:size(waypoints,2)
% 
%    
%     % sense 
%     [pcCurrent,img,q] = sense(robot,ROSobjects,pcRoi);
%     [BW,maskedImage] = segmentImage(img);
%     % estimate enviroment
%     pcWorld = updateWorld(robot, ROSobjects,pcCurrent,pcWorld);
%     
%     % segment
%     ;
%     % identify 
%      
%     pos_move = [0 0 -0.1];
%     rot_move = [0 0 -pi / 10];
% 
%     MTH_target = waypoints{k};
%     % plot
%     run plotInfo.m
%     
%     % move 
%     moveTo(robot, q,MTH_target, ROSobjects)
%     
%     % display info
%     disp("Number of clusters " + numClusters)
% end
