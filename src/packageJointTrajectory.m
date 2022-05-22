function msg = packageJointTrajectory(msg,q,qd,qdd,trajTimes,tolerance)
%   Copyright 2021 The MathWorks, Inc
%
%   This function packages the joint positions, velocities, and
%   accelerations of the desired trajectory into ROS messages as required
%   by ros_control for inetraction with the Gazebo environment
%   tolerance: vector 1x3 [position, velocity, acceleration] tolerance
   
    % Initialize values
    N = numel(trajTimes);
    numJoints = size(q,1);
    zeroVals = zeros(numJoints,1);
    jointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7'};
    % Assign joint names to ROS message
    msg.Trajectory.JointNames = jointNames;
    
    % Assign constraints
    for idx = 1:numJoints
        msg.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance');
        msg.GoalTolerance(idx).Name = jointNames{idx};
        msg.GoalTolerance(idx).Position = tolerance(1);
        msg.GoalTolerance(idx).Velocity = tolerance(2);
        msg.GoalTolerance(idx).Acceleration = tolerance(3);
    end
    
    % Loop through waypoints and fill in their data
    %trajPts(N) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    for idx = 1:N
        m = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        m.TimeFromStart = rosduration(trajTimes(idx));
        m.Positions = q(:,idx);
        m.Velocities = qd(:,idx);
        m.Accelerations = qdd(:,idx);
        m.Effort = zeroVals;
        trajPts(idx) = m;
        % Uncomment below to display packaging progress
%         if mod(idx,round(N/10))==0
%            disp(['Packing waypoint ' num2str(idx) ' of ' num2str(N) '...']); 
%         end
    end  
    msg.Trajectory.Points = trajPts;    
end
