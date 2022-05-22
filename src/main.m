clear, clc, close all
%% Connect to ROS Network
device = rosdevice('localhost');

runfromMATLAB = false;
 
if ~isCoreRunning(device) && runfromMATLAB % run roslaunch ROSdistribution: Noetic
    bashConfig='source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash';
    bashLibraries = 'export LD_LIBRARY_PATH="~/catkin_ws/devel/lib:/opt/ros/noetic/lib"';
    bashRunGazebo = 'roslaunch kortex_gazebo_depth pickplace.launch world:=RoboCup_1.world';    
    [status,cmdout] = system([bashConfig ';' bashLibraries ';' bashRunGazebo '&  echo $!'])
    % system([ba    shConfig ';' bashLibraries ';' bashRunGazebo])
    % system(['kill' cmdout]);   % end ros process 
    % system('killall -9 -v rosmaster')
    pause(7)
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
setInitialConfig();

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
% sense ->  estimate environment -> segment -> identify-> decide -> move -> repeat
[ptCloud, msh] = estimate(robot, ROSobjects);

current_joint_state = ROSobjects.jntStateSub.LatestMessage;
q = current_joint_state.Position(2:8);

close all
figure(1)
show(robot,q')  
hold on
pcshow(ptCloud)
axis tight

% %%
% tic
% [msg,xyzGlobal,labels,numClusters, q_m,MTH_target] = algorithm(ROSobjects,trajeGoalMsg,robot,ptCloudGlobal);
% time_spent=toc;
% sendGoalAndWait(ROSobjects.trajeAct,msg)
% 
% %% Plot
% % plot point cloud
% close all
% 
% subplot(2,1,1)
% pcshow(xyzGlobal,labels,'MarkerSize',10)
% hold on
% %show(m)
% show(robot,q_m')
% hold off
% disp("Number of clusters " + numClusters)
% view(0,90)
% 
% subplot(2,2,3)
% pcshow(xyzGlobal,labels,'MarkerSize',10)
% hold on
% %show(m)
% show(robot,q_m')
% hold off
% disp("Number of clusters " + numClusters)
% view(0,0)
% 
% subplot(2,2,4)
% pcshow(xyzGlobal,labels,'MarkerSize',10)
% hold on
% %show(m)
% show(robot,q_m')
% hold off
% disp("Number of clusters " + numClusters)
% view(90,0)
% % subplot(2,1,1)
% % imshow(depth)
% % subplot(2,1,2)
% % imshow(img)