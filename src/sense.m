function ptCloud = sense(robot,ROSobjects)

ImgSub = ROSobjects.ImgSub;
ptcSub = ROSobjects.ptcSub;
jntStateSub = ROSobjects.jntStateSub;

% Receive images
img = readImage(receive(ImgSub));
xyz = rosReadXYZ(receive(ptcSub));

% Receive joint states
current_joint_state = receive(jntStateSub);
q = current_joint_state.Position(2:8);

ptCloud = getPointCloud(xyz, img, robot, q');
end
