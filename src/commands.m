
%gen3_joint_trajectory_controller, joint_state_controller, custom_gripper_controller
%% get information about basic ROS components
rosnode list

rostopic list

rosservice list

rosmsg list

rostopic info /scan

rostopic info /camera/color/image_raw 
rostopic info /camera/depth/camera_info
%% create a subscriber and recieve data
topic='/camera/color/image_raw/compressed'

sub= rossubscriber(topic)

data=receive(sub,1)
% /camera/color/image_raw/compressedDepth  
% /camera/depth/camera_info                                                         
% /camera/depth/image_raw                                                           
% /camera/depth/points  



%% ROS publisher
% send(pub,msg)
% rosplot
%% ROS message

% msg = rosmessage(messagetype) 
% msg = rosmessage(sub)
rosmsg show geometry_msgs/Twist

%% ROS service
time = 1
client = rossvcclient('/my_gen3/controller_manager/load_controller')

% waitForServer(client,'Timeout',time)
% response=call(client,'Timeout',3) 
% shell: rossrv list

