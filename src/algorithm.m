function [trajGoalMsg,xyzGlobal,labels,numClusters, q_m,MTH_target] = algorithm(ROSNodes,trajGoalMsg,robot,ptCloudGlobal)
    
    joint_state_sub  = ROSNodes.joint_state_sub; 
    trajAct = ROSNodes.trajAct;
    ImgSub = ROSNodes.ImgSub; 
    ptcSub = ROSNodes.ptcSub ;

    %% sense
    % get data from gazebo
    curImage = receive(ImgSub);
    img =readImage(ImgSub.LatestMessage);
    
    xyz = rosReadXYZ(receive(ptcSub));
    
    get_joint_msg = receive(joint_state_sub,1);
    q_m = get_joint_msg.Position(2:8);
    % sensed data: xyz, img, q_m
    %% estimate enviroment
    % pointcloud  enviroment estimation
    [ptCloudSegment,MTH] = getPointCloud(xyz,img,robot,q_m);
    gridStep = 0.01;
    ptCloudGlobal = pcmerge(ptCloudGlobal,ptCloudSegment,gridStep);
    %% segment 
    distance = 0.05;
    [labels, numClusters] = pcsegdist(ptCloudGlobal,distance);
    
    %% identify 
    % can and bottle classification
    
    %% decide 
    % create collision mesh
    xyzGlobal = ptCloudGlobal.Location;
    
    m = collisionMesh(xyzGlobal);
    ik = inverseKinematics('RigidBodyTree',robot);
    pos_move=[0 0 -0.1] ;
    rot_move= [0 0 -pi/10];
    
    blueBinPose = trvec2tform(pos_move)*eul2tform(rot_move);
    greenBinPose = trvec2tform(pos_move)*eul2tform(rot_move);
    
    MTH_target= trvec2tform(pos_move)*eul2tform(rot_move)*MTH;
    weights = [0.25 0.25 0.25 1 1 1];
    initialguess = q_m';
    [configSoln,solnInfo] = ik('camera',MTH_target,weights,initialguess);
    
    
    target = configSoln';
    
    %% Robot move
    %RoboCupManipulation_setInitialConfig;
    
    zeroVals = zeros(7,1);
    q = target;
    qd = zeroVals;
    qdd = zeroVals;
    trajTimes = 2;
    tolerance = [0.3 0.1 0.1];
    trajGoalMsg = packageJointTrajectory(trajGoalMsg,q,qd,qdd,trajTimes,tolerance)


end