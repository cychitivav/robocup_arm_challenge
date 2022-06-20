function [q,pcWorld,img,BW,depth] = sense(robot,ROSobjects,pcRoi)

ImgSub = ROSobjects.ImgSub;
DepthSub = ROSobjects.DepthSub;
ptcSub = ROSobjects.ptcSub;
jntStateSub = ROSobjects.jntStateSub;

% Receive images
img = readImage(receive(ImgSub));
BW = segmentImage(img);
depth = readImage(receive(DepthSub));

xyzLocal = rosReadXYZ(receive(ptcSub));
roi_xyz=reshape(permute(BW, [2, 1]), [],1 );


% Clean data
rowInvalid = or(any(isnan(xyzLocal), 2), ~ roi_xyz);
xyzLocal(rowInvalid, :) = [];
xyzLocal = double(xyzLocal);

% Transform point cloud to base_link frame
current_joint_state = receive(jntStateSub);
q = current_joint_state.Position(2:8);

MTH = getTransform(robot, q', 'camera', 'base_link');

xyzHomo = [xyzLocal ones(size(xyzLocal, 1), 1)]';
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
