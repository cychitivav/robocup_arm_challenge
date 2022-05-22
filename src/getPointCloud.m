function ptCloudSegment = getPointCloud(xyz,img,robot,q)
% get pointcloud a tranformation based on the robot 
%
% ptCloudSegment=getPointCloud(xyz,img,robot,q)
%
% xzy: point cloud location
% img: image same size as pointcloud
% robot: robot object
% q: robot joint state

% clean data  
invalid=any(isnan(xyz),2);
xyz(invalid,:)=[];
xyz=double(xyz);
cdata=reshape(permute(img,[2,1,3]),[],3);
cdata(invalid,:)=[];

q = q(:)';

%data to pointcloud
ptCloud =  pointCloud(xyz);
ptCloud.Color= cdata;

MTH = getTransform(robot,q,'camera')
%pose_prima=MTH*pose;

rot=MTH(1:3,1:3);
trans=MTH(1:3,4)';
tform = rigid3d(rot,trans);
pose = [xyz zeros(size(xyz,1),1)]'; 
ptCloudSegment = pctransform(ptCloud,tform);

end