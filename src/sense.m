function [pcWorld,img,q] = sense(robot,ROSobjects,pcRoi)

ImgSub = ROSobjects.ImgSub;
ptcSub = ROSobjects.ptcSub;
jntStateSub = ROSobjects.jntStateSub;

% Receive images
img = readImage(receive(ImgSub));
xyz = rosReadXYZ(receive(ptcSub));

% Clean data
rowInvalid = any(isnan(xyz), 2);
xyz(rowInvalid, :) = [];
xyz = double(xyz);

% Transform point cloud to base_link frame
current_joint_state = receive(jntStateSub);
q = current_joint_state.Position(2:8);

MTH = getTransform(robot, q', 'camera', 'base_link');

xyzHomo = [xyz ones(size(xyz, 1), 1)]';
xyzHomoWorld = MTH * xyzHomo;

xyzWorld = xyzHomoWorld(1:3, :)';
pcWorld = pointCloud(xyzWorld);

% Extract points in ROI
indices = findPointsInROI(pcWorld,pcRoi);
pcWorld = pointCloud(xyzWorld(indices,:));

% Add color to point cloud
colorData = reshape(permute(img, [2, 1, 3]), [], 3);
colorData(rowInvalid, :) = [];

pcWorld.Color = colorData(indices,:);

end
