function [ptCloudSegment, MTH] = getPointCloud(xyz, img, robot, q)
% GETPOINTCLOUD returns the point cloud in the base_link frame
% of the robot.
%   ptCloudSegment = GETPOINTCLOUD(xyz,img,robot,q)
%
%   xzy: point cloud location
%   img: image same size as pointcloud
%   robot: robot object
%   q: robot joint state

% Clean data
rowInvalid = any(isnan(xyz), 2);
xyz(rowInvalid, :) = [];
xyz = double(xyz);

colorData = reshape(permute(img, [2, 1, 3]), [], 3);
colorData(rowInvalid, :) = [];

% Data to pointcloud
ptCloud = pointCloud(xyz);
ptCloud.Color = colorData;

% Transform pointcloud to base_link frame
q = q(:)';
MTH = getTransform(robot, q, 'camera');

rotm = MTH(1:3, 1:3);
trans = MTH(1:3, 4);
tform = rigid3d(rotm, trans');

ptCloudSegment = pctransform(ptCloud, tform);
end
