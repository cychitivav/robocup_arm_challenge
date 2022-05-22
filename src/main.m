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
rosinit %('127.0.0.1',11311)

%% Load robot model
load('exampleHelperKINOVAGen3GripperROSGazebo.mat');
bodyCam = rigidBody('camera');
addBody(robot,bodyCam,'gripper')
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
ptcSub = rossubscriber('/camera/depth/points','DataFormat','struct')

%% Test image processing
close all
% get data from gazebo
curImage = receive(ImgSub);
img =readImage(ImgSub.LatestMessage);

curDepth = receive(DptSub);
depth = readImage(DptSub.LatestMessage); 

xyz = rosReadXYZ(receive(ptcSub));

% clean data  
invalid=any(isnan(xyz),2);
xyz(invalid,:)=[];
xyz=double(xyz);
cdata=reshape(permute(img,[2,1,3]),[],3);
cdata(invalid,:)=[];

%data to pointcloud
ptCloud =  pointCloud(xyz);
ptCloud.Color= cdata;

% reference frame transformation
get_joint_msg = receive(joint_state_sub,1);
q_m = get_joint_msg.Position(2:8);

MTH = getTransform(robot,q_m','camera')
%pose_prima=MTH*pose;

rot=MTH(1:3,1:3);
trans=MTH(1:3,4)';
tform = rigid3d(rot,trans);
pose = [xyz zeros(size(xyz,1),1)]'; 
ptCloudSegment = pctransform(ptCloud,tform);
%%

ptCloudGlobal = ptCloudSegment;
gridStep = 0.01;
ptCloudGlobal = pcmerge(ptCloudGlobal,ptCloudSegment,gridStep);
%% create collision mesh
m = collisionMesh(ptCloudGlobal.Location);

%% plot point cloud
close all

pcshow(ptCloudGlobal,'MarkerSize',10)
hold on
%show(m)

show(robot,q_m')
hold off


%% Robot move
%RoboCupManipulation_setInitialConfig;

zeroVals = zeros(7,1);
get_joint_msg = receive(joint_state_sub,1);
q_m = get_joint_msg.Position(2:8);
q = q_m-[0.5 0 0 0 0 0 0]';
qd = zeroVals;
qdd = zeroVals;
t=rostime('now');
trajTimes = 2;
msg = packageJointTrajectory(trajGoalMsg,q,qd,qdd,trajTimes)
waitForServer(trajAct);
sendGoalAndWait(trajAct,msg)

%% send trajectory commands

m = rosmessage('trajectory_msgs/JointTrajectoryPoint')
%rostopic  info /my_gen3/joint_states

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