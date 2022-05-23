function ptCloudWorld = getPointCloud(xyz, img, robot, q)
% GETPOINTCLOUD returns the point cloud in the base_link frame
% of the robot.
%   ptCloud = GETPOINTCLOUD(xyz,img,robot,q)
%
%   xzy: point cloud location
%   img: image same size as pointcloud
%   robot: robot object
%   q: robot joint state
%   ptCloud: point cloud respect to base_link frame

% Clean data
rowInvalid = any(isnan(xyz), 2);
xyz(rowInvalid, :) = [];
xyz = double(xyz);

colorData = reshape(permute(img, [2, 1, 3]), [], 3);
colorData(rowInvalid, :) = [];

% Data to point cloud
ptCloud = pointCloud(xyz);
ptCloud.Color = colorData;

% Transform point cloud to base_link frame
MTH = getTransform(robot, q, 'camera', 'base_link');

pose = [xyz ones(size(xyz, 1), 1)]';
poseWorld = MTH * pose;

ptCloudWorld = pointCloud(poseWorld(1:3, :)');
ptCloudWorld.Color = colorData;
end
